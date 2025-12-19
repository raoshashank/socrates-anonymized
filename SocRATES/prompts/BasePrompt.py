from utils.utils import *
class BasePrompt:
    def __init__(self,model_name = 'gpt-3.5-turbo') -> None:
        self.system_prompt = "You are a helpful assistant designed to output JSON"
        self.examples = ""
        self.full_prompt = ""
        self.imgs = []        
    
    def get_full_prompt(self,**kwargs):
        return self.system_prompt + "\n" + self.examples + "\n" + self.full_prompt
    
    # def get_text_payload(self,**kwargs):
    #     payload = [
    #     {"role":"system",
    #      "content": self.system_prompt},
    #     {"role": "user",
    #     "content": [
    #         {"type": "text", 
    #          "text": self.get_full_prompt(**kwargs)}]}]
    #     return payload 
    
    # def get_img_payload(self,**kwargs):
    #     payload = [
    #     {   "role":"system",
    #         "content": self.system_prompt},
    #         {
    #         "role": "user",
    #         "content": [
    #         {"type": "text",
    #          "text": self.get_full_prompt(**kwargs)}],
    #     }
    #     ]
    #     for i in range(len(self.imgs)):
    #         payload[1]["content"].append({"type": "image_url", 
    #                                       "image_url": {"url":f"data:image/jpeg;base64,{list(self.imgs.values())[i]}",
    #                                                     "detail":"high"}})
        
    #     return payload
