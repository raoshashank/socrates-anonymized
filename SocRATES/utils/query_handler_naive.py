from click import edit
from utils.llm_models import models
from utils.scene_graph import SceneGraph
from prompts import *
from utils import utils
from utils.utils import *
import xml.etree.ElementTree as ET
import json
import tenacity

class QueryHandler:
    def __init__(self,config) -> None:
        self.retry_count = config['retry_count']
        self.model = models[config['model']['model_type']](
            model_name = config['model']['model_name'],
            debug = config['debug'])
        
        self.debug = config['debug']
    def query_scenario(self,**kwargs):
        '''
        Sends a scenario query to the LLM and returns the response.
        '''
        scQ = AbalationQuery(
            scene_graph = kwargs['scene_graph'],
            node_types = kwargs['node_types'],
            edge_types = kwargs['edge_types'],
            encoded_img = kwargs['encoded_img']
        )
        
        #generate full prompt
        scQ_full_prompt = scQ.get_full_prompt(
            context=kwargs['context'],
            task=kwargs['task'],
            rough_scenario=kwargs['rough_scenario'],
            location = kwargs['location_description']
        )
        lprint("Querying LLM for scenario")
        try:
            #retry N times
            for attempt in tenacity.Retrying(stop=tenacity.stop_after_attempt(self.retry_count),
                                                wait=tenacity.wait_random_exponential(multiplier=1,max=40)):
                with attempt:
                    scq_response = self.model.get_response(messages = scQ_full_prompt,response_format= StructuredResponse)
                    scq_response_structured = self.model.extract_response(scq_response)
                    if not scq_response_structured:
                        #check if 'human_behavior' key is a dict with N keys
                        raise ValueError('Invalid response, retrying')
        except tenacity.RetryError:
            eprint("Failed to generate scenario")
            return None           
        return scq_response_structured
        
               
        