from openai import OpenAI
import utils.utils as utils
import xml.etree.ElementTree as ET
def error_callback(response,retry_reason,required_keys):
    if (retry_reason == 'json'):
        return [
            {
                "role":"assistant",
                "content":[
                    {
                        "type":"text",
                        "text": response
                    }
                ]
            },
            {
                "role":"user",
                "content":[
                    {
                        "type":"text",
                        "text": "Your output is not parseable with python's json.loads function. Please try again."
                    }
                ]
            }
        ]
    elif(retry_reason == 'keys'):
        return [
            {
                "role":"assistant",
                "content":[
                    {
                        "type":"text",
                        "text": response
                    }
                ]
            },
            {
                "role":"user",
                "content":[
                    {
                        "type":"text",
                        "text": f"Your output is does not have the correct output keys. Your output should be parseable by the json.loads python function and have the following keys:{required_keys}"
                    }
                ]
            }
        ]
    elif(retry_reason == 'xml'):
        return [
            {
                "role":"assistant",
                "content":[
                    {
                        "type":"text",
                        "text": response
                    }
                ]
            },
            {
                "role":"user",
                "content":[
                    {
                        "type":"text",
                        "text": "Your output is not a valid XML tree. It is not parseable with python's xml.etree.ElementTree.fromstring function. Please try again."
                    }
                ]
            }
        ]
    else:
        return []

class GPTModel:
    def __init__(self,model_name,tools = None,debug = False,seed = 42) -> None:
        self.model_name = model_name
        #self.openai_temperature = config.OPENAI_TEMPERATURE
        self.client=OpenAI()
        self.messages = []
        self.tools = tools      
        self.debug = debug
        self.seed = seed
    
    def get_response(self,messages,response_format):
        return self.client.beta.chat.completions.parse(
            model = self.model_name,
            messages = messages,
            response_format = response_format, #use structured format,
            seed=self.seed
        )

    def extract_response(self,response):
        output = response.choices[0].message
        if response.choices[0].finish_reason != 'stop' or output.refusal:
            return False
        return output.parsed
    
    def extract_usage_stats(self,response):
        return {
            'prompt_tokens': response.usage.prompt_tokens,
            'completion_tokens': response.usage.completion_tokens,
            'total_tokens': response.usage.total_tokens
        }
    
class InstructorResponse:
    """Lightweight container so InstructorModel matches the GPTModel interface:
    a single object that is passed to extract_response / extract_usage_stats."""
    def __init__(self, parsed, completion):
        self.parsed = parsed
        self.completion = completion


# Providers that accept the OpenAI-style `seed` parameter for deterministic
# sampling. Other providers reject unknown kwargs, so seed is only forwarded
# for these.
_SEED_SUPPORTED = {"openai", "azure_openai", "groq"}

# Providers that require an explicit max_tokens on every request.
_MAXTOKENS_REQUIRED = {"anthropic", "bedrock_anthropic"}
_DEFAULT_MAX_TOKENS = 4096

# Providers that natively accept OpenAI-style content blocks
# ([{"type": "text", ...}, {"type": "image_url", ...}]). For every other
# provider the prompts' OpenAI-style blocks are normalized (see below) into the
# provider-neutral form instructor understands.
_OPENAI_CONTENT_NATIVE = {"openai", "azure_openai"}


def _normalize_message_content(content):
    """Convert OpenAI-style content blocks into the provider-neutral form
    instructor accepts across backends: a plain string, or a list mixing
    strings and ``instructor.Image`` objects (which instructor renders per
    provider). Leaves plain-string content untouched."""
    if content is None or isinstance(content, str):
        return content
    import instructor
    items = []
    for block in content:
        if isinstance(block, str):
            items.append(block)
            continue
        if not isinstance(block, dict):
            items.append(block)
            continue
        btype = block.get("type")
        if btype == "text":
            items.append(block.get("text", ""))
        elif btype == "image_url":
            img = block.get("image_url")
            url = img.get("url") if isinstance(img, dict) else img
            items.append(instructor.Image.autodetect(url))
        elif btype in ("image", "input_image"):
            src = block.get("image") or block.get("source") or block.get("url")
            items.append(instructor.Image.autodetect(src)
                         if isinstance(src, str) else block)
        else:
            items.append(block.get("text", str(block)))
    # Collapse a lone text item to a plain string (most compatible for system
    # and simple user messages).
    if len(items) == 1 and isinstance(items[0], str):
        return items[0]
    return items


# Per-provider role remapping. Gemini's conversation roles are only "user" and
# "model" (the system message is handled separately), so the few-shot/retry
# "assistant" turns must be relabeled "model". Other providers use "assistant".
_ROLE_MAP = {
    "google": {"assistant": "model"},
    "gemini": {"assistant": "model"},
    "vertexai": {"assistant": "model"},
}


def _normalize_messages(messages, provider=None):
    role_map = _ROLE_MAP.get(provider, {})
    out = []
    for msg in messages:
        if isinstance(msg, dict) and "content" in msg:
            new = dict(msg)
            if new.get("role") in role_map:
                new["role"] = role_map[new["role"]]
            new["content"] = _normalize_message_content(msg["content"])
            out.append(new)
        else:
            out.append(msg)
    return out


class InstructorModel:
    """Provider-agnostic structured-output model backed by the `instructor`
    library. Works with any provider instructor supports (openai, anthropic,
    google, mistral, groq, ollama, ...) while exposing the same interface as
    GPTModel: get_response / extract_response / extract_usage_stats.

    Provider and model are selected with a single "provider/model" string,
    e.g. "openai/gpt-4.1", "anthropic/claude-opus-4-8",
    "google/gemini-2.5-flash".
    """

    def __init__(self, model_name, provider="openai", tools=None, debug=False,
                 seed=None, temperature=None, max_tokens=None, max_retries=1,
                 extra_kwargs=None, **kwargs):
        try:
            import instructor
        except ImportError as e:
            raise ImportError(
                "The 'instructor' package is required for InstructorModel. "
                "Install it with `pip install instructor` and the provider SDK "
                "you intend to use (e.g. `pip install anthropic` or "
                "`pip install google-genai`)."
            ) from e

        self.provider = provider
        self.model_name = model_name
        self.debug = debug
        self.seed = seed
        self.temperature = temperature
        if max_tokens is None and provider in _MAXTOKENS_REQUIRED:
            max_tokens = _DEFAULT_MAX_TOKENS
        self.max_tokens = max_tokens
        self.max_retries = max_retries
        self.extra_kwargs = dict(extra_kwargs) if extra_kwargs else {}
        self.tools = tools
        # instructor binds the model into the client, so the model is not
        # passed again on each request.
        self.client = instructor.from_provider(f"{provider}/{model_name}")

    def _request_kwargs(self):
        kwargs = dict(self.extra_kwargs)
        if self.temperature is not None:
            kwargs.setdefault("temperature", self.temperature)
        if self.max_tokens is not None:
            kwargs.setdefault("max_tokens", self.max_tokens)
        if self.seed is not None and self.provider in _SEED_SUPPORTED:
            kwargs.setdefault("seed", self.seed)
        return kwargs

    def get_response(self, messages, response_format):
        # `response_model` replaces OpenAI's `response_format`; instructor
        # validates the output against it (raising on failure after retries).
        # For non-OpenAI providers the prompts' OpenAI-style content blocks are
        # normalized to instructor's neutral string/Image form; `autodetect_images`
        # additionally converts any plain image URL/path strings per provider.
        if self.provider not in _OPENAI_CONTENT_NATIVE:
            messages = _normalize_messages(messages, self.provider)
        parsed, completion = self.client.chat.completions.create_with_completion(
            messages=messages,
            response_model=response_format,
            max_retries=self.max_retries,
            autodetect_images=True,
            **self._request_kwargs(),
        )
        return InstructorResponse(parsed, completion)

    def extract_response(self, response):
        # instructor validates against response_format and raises on failure,
        # so a returned response always carries a valid parsed object.
        return response.parsed

    def extract_usage_stats(self, response):
        # Normalize usage across providers:
        #   OpenAI:    usage.prompt_tokens / completion_tokens / total_tokens
        #   Anthropic: usage.input_tokens / output_tokens
        #   Gemini:    usage_metadata.prompt_token_count / candidates_token_count
        completion = response.completion
        usage = (getattr(completion, "usage", None)
                 or getattr(completion, "usage_metadata", None))
        if usage is None:
            return {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0}
        prompt = (getattr(usage, "prompt_tokens", None)
                  or getattr(usage, "input_tokens", None)
                  or getattr(usage, "prompt_token_count", None) or 0)
        completion_tokens = (getattr(usage, "completion_tokens", None)
                             or getattr(usage, "output_tokens", None)
                             or getattr(usage, "candidates_token_count", None) or 0)
        total = (getattr(usage, "total_tokens", None)
                 or getattr(usage, "total_token_count", None)
                 or (prompt + completion_tokens))
        return {
            "prompt_tokens": prompt,
            "completion_tokens": completion_tokens,
            "total_tokens": total,
        }


# Registry of model backends keyed by the config `provider` / `model_type`.
# "gpt"/"gpt-native" use OpenAI's native structured-output parse (GPTModel);
# everything else is routed through instructor (InstructorModel).
models = {
    'gpt': GPTModel,
    'gpt-native': GPTModel,
    'instructor': InstructorModel,
}


def build_model(model_cfg, debug=False, seed=None):
    """Construct the right model backend from a config block.

    The block selects a backend with `provider` (preferred) or the legacy
    `model_type` key:
        provider: "openai" | "anthropic" | "google" | "mistral" | "groq" | ...
                  -> InstructorModel (provider-agnostic, via instructor)
        provider: "gpt" | "gpt-native"
                  -> GPTModel (OpenAI native structured-output parse)
    Optional per-model keys: temperature, max_tokens, max_retries, extra_kwargs.
    """
    provider = model_cfg.get('provider') or model_cfg.get('model_type') or 'openai'
    model_name = model_cfg['model_name']

    if provider in ('gpt', 'gpt-native'):
        return GPTModel(model_name=model_name, debug=debug, seed=seed)

    extra = model_cfg.get('extra_kwargs')
    return InstructorModel(
        provider=provider,
        model_name=model_name,
        debug=debug,
        seed=seed,
        temperature=model_cfg.get('temperature'),
        max_tokens=model_cfg.get('max_tokens'),
        max_retries=model_cfg.get('max_retries', 1),
        extra_kwargs=dict(extra) if extra else None,
    )
