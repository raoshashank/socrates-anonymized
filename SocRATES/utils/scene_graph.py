import networkx as nx
import random
class SceneGraph:
    def __init__(self, serialized_graph):
        '''
        Nodes have attributes:
            - type
            - ID (name)
            - position (pixel)
        '''
        self.graph = nx.node_link_graph(serialized_graph) #nx graph
        nx.set_edge_attributes(self.graph, {e: self.dist(e[0],e[1]) for e in self.graph.edges()}, "cost")            

    def get_parent_nodes(self):
        '''
        This function returns the parent nodes in the graph
        '''
        return [n for n in self.graph.nodes if self.graph.nodes[n]['type']!='child']

    def dist(self,n1,n2):
        (x1,y1) = self.graph.nodes[n1]['pos']
        (x2,y2) = self.graph.nodes[n2]['pos']
        return ((x1-x2)**2+(y1-y2)**2)**0.5        

    def sampleNodeOfType(self,node_type):
        '''
        This function samples nodes in the graph that are of the input type
        '''
        nodes = [n for n in self.graph.nodes if self.graph.nodes[n]['type']==node_type]
        if len(nodes)==0:
            return f"No node of type {node_type} exists in the graph"
        return random.choice(nodes)
    '''   
    def relativeDirection(self,node1,node2):
        
        #This function returns the relative direction between two nodes.
        #for example, 'right,below' means that node1 is to the right and below node2
        
        try:
            (x1,y1) = self.graph.nodes[node1]['pos']
            (x2,y2) = self.graph.nodes[node2]['pos']
        except KeyError:
            return "These nodes don't exist in the graph"
        direction = [None,None]
        if x1>=x2:
            direction[0] = 'right'
        elif x1<=x2:
            direction[0] = 'left'
        elif y1>=y2:
            direction[1] = 'below'
        elif y1<=y2:
            direction[1] = 'above'
        else:
            return 'None'
        return direction
    '''
    def connectedNodes(self,node):
        '''
        This function returns all the neighbours of the input node and their types and distance from the node
        '''
        cnodes = 'None'
        try:
            neighbours =  list(self.graph.neighbors(node))
            cnodes = {}
            for c in neighbours:
                cnodes[self.graph.nodes[c]] = self.graph.edges[(node,c)]['cost']
        except nx.NetworkXError:
            return "No such node exists in the graph"
        return cnodes
    
    
    def planPath(self,node1,node2):
        '''
        This function finds the shortest path between node1 and node2
        '''
        # returns -1 if no path exists
        cost = 0
        try:
            path =  nx.astar_path(self.graph,node1,node2, weight = "cost")
            n1 = path[0]
            if len(path) > 1:
                for n in path[1:]:
                    cost += self.graph.edges[n1,n]["cost"]
                    n1 = n
        except nx.NetworkXNoPath:
            cost = -1
        return cost

    def areTrajectoriesIntersecting(self,traj1,traj2):
        #checks if two trajectories have at least 1 node in common
        #input: traj1 and traj2 are sequences of node names in the graph
        for n1 in traj1:
            for n2 in traj2:
                if n1 == n2:
                    return True
        return False
    
    def isvalidtrajectory(self,trajectory):
        #checks if a given node sequence is valid or not and returns the errors
        #input: trajectory: sequence of node names in the graph
        errors = []
        traj_valid = True
        if len(trajectory) == 0:
            return False,()
        
        for first,second in zip(trajectory,trajectory[1:]):
            if (first,second) not in self.graph.edges:
                errors.append((first,second))
                traj_valid = False
        return traj_valid, errors