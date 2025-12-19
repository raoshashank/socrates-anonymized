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
    
models = {
    'gpt':GPTModel
}