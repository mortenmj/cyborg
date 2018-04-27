#!/usr/bin/env python

import b3
import json
import rospy

from cyborg_msgs.msg import BehaviorTreeNodes
from cyborg_msgs.srv import BehaviorTree, BehaviorTreeResponse
from cyborg_bt_nodes.actions import MoveTo
import networkx as nx


NAME = 'behavior_tree_manager'


class BehaviorTreeManager():
    def __init__(self):
        rospy.init_node(NAME)

        rospy.loginfo('Initializing: %s' % NAME)

        names = {'MoveTo': MoveTo}

        with open('/home/mortenmj/.ros/project.json') as f:
            rospy.loginfo('Loading project from %s' % f.name)
            data = json.load(f)

        self.root = self.load_project(data, names)
        if not self.root:
            raise AttributeError

        self.target = None
        self.blackboard = b3.Blackboard()
        self.open_nodes = list()

        # Set known locations
        locations = {
                'el5':         [-33.768, -33.545, 0],
                'waitingarea': [-33.581,  10.627, 0],
                'info':        [-33.505,  1.139, 0],
                'cafeteria':   [-33.075, -55.776, 0],
                'stairs':      [-31.494, -71.056, 0],
                'el6':         [-30.397, -12.826, 0],
                'hallway':     [-30.251, -79.089, 0],
                'home':        [-29.552,  8.747, 0],
                'elevator1':   [-29.414, -50.281, 0],
                'underbridge': [-28.161, -63.308, 0],
                'elevator2':   [-21.912, -42.451, 0],
                'entrance':    [-18.422,  6.518, 0],
                'entrance2':   [-18.246, -65.898, 0]
        }

        self.blackboard.set('locations', locations)

        rospy.loginfo('Running %s' % self.root)

    def __str__(self):
        if not hasattr(self, '_dot'):
            rospy.loginfo("Creating dot repr")
            self._dot = self.to_pydot(self.root).to_string()
        return self._dot

    def bt_cb(self, data):
        """
        Return DOT representation of the behavior tree
        """
        G = self.to_networkx(self.root)
        tree = json.dumps(G)

        response = BehaviorTreeResponse()
        response.tree = tree

        return response

    def load_project(self, data, names=None):
        names = names or {}
        trees = {}

        rospy.loginfo('Creating BehaviorTree objects')

        # Create the BehaviorTree objects
        for tree in data['trees']:
            tmptree = b3.BehaviorTree()

            tmptree.id = tree['id'] or tree.id
            tmptree.title = tree['title'] or tree.title
            tmptree.description = tree['description'] or tmptree.description
            tmptree.properties = tree['properties'] or tmptree.properties

            trees[tree['id']] = tmptree
            names[tree['id']] = tmptree

        rospy.loginfo('Populating BehaviorTrees with nodes')

        # Create the nodes in each tree
        for tree in data['trees']:
            rospy.loginfo('Creating nodes for: %s' % tree['title'])

            nodes = tree['nodes']
            tmpnodes = {}

            for key in nodes:
                spec = nodes[key]
                name = spec['name']

                rospy.loginfo('Creating node: %s' % name)

                if name in trees:
                    # Node is a tree
                    tmpnodes[key] = trees[name]
                else:
                    if name in names:
                        cls = names[name]
                    elif hasattr(b3, name):
                        cls = getattr(b3, name)
                    else:
                        rospy.logerror('Invalid node name: %s' % name)
                        AttributeError('BehaviorTree.load_project: Invalid node name "%s"' % name)

                    params = spec['properties']
                    node = cls(**params)
                    node.id = spec['id'] or node.id
                    node.title = spec['title'] or node.title
                    node.description = spec['description'] or node.description
                    node.properties = spec['properties'] or node.properties
                    tmpnodes[key] = node

            rospy.loginfo('Connecting nodes')

            # Connect the nodes
            for key in nodes:
                spec = nodes[key]
                node = tmpnodes[key]

                if node.category == b3.COMPOSITE and 'children' in spec:
                    for cid in spec['children']:
                        node.children.append(tmpnodes[cid])
                elif node.category == b3.DECORATOR and 'child' in spec:
                    node.child = tmpnodes[spec['child']]

            rospy.loginfo('Connecting root node')

            # Connect the root node for the tree
            trees[tree['id']].root = tmpnodes[tree['root']]

        root_tree = trees[data['selectedTree']]

        return root_tree

    def to_networkx(self, tree):
        """
        Generate a networkx graph.

        Parameters
        ----------
        tree: the behavior tree

        Returns
        -------
        networkx.DiGraph: graph
        """
        from networkx.readwrite import json_graph

        def process_tree(G, tree):
            root = tree.root
            G.set_name = tree.id
            G.add_node(root.id, label=str(root))
            process_node(G, root)

        def process_node(G, node):
            if isinstance(node, b3.Composite):
                for c in node.children:
                    if isinstance(c, b3.BehaviorTree):
                        c = c.root

                    rospy.loginfo('category: %s' % c.category)
                    G.add_node(
                            c.id,
                            label=str(c),
                            category=str(c.category))
                    G.add_edge(node.id, c.id)
                    process_node(G, c)

        G = nx.DiGraph()
        process_tree(G, tree)

        return json_graph.tree_data(G, root=tree.root.id)

    def to_pydot(self, tree):
        """
        Generate a pydot graph.

        Parameters
        ----------
        tree: the behavior tree

        Returns
        -------
        pydot.Dot: graph
        """
        import pydot

        def process_tree(graph, tree):
            root = tree.root
            graph.set_name = tree.title
            graph.add_node(root.graph_node)
            process_node(graph, root)

        def process_node(graph, node):
            if isinstance(node, b3.Composite):
                for c in node.children:
                    if isinstance(c, b3.BehaviorTree):
                        subgraph = pydot.Subgraph(c.title)
                        graph.add_subgraph(subgraph)
                        graph.add_edge(node.graph_edge(c.root))
                        process_tree(subgraph, c)
                    else:
                        graph.add_node(c.graph_node)
                        graph.add_edge(node.graph_edge(c))
                        process_node(graph, c)

        graph = pydot.Dot(graph_type='digraph')
        process_tree(graph, tree)

        return graph

    def run(self):
        pub_name = '/cyborg/bt/behavior_tree_updates'
        pub = rospy.Publisher(pub_name, BehaviorTreeNodes, latch=True, queue_size=1)

        srv_name = 'cyborg/bt/get_behavior_tree'
        rospy.Service(srv_name, BehaviorTree, self.bt_cb)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            open_nodes = self.blackboard.get('open_nodes', self.root.id)

            if set(open_nodes) != set(self.open_nodes):
                self.open_nodes = open_nodes
                pub.publish([node.id for node in self.open_nodes])

            self.root.tick(self.target, self.blackboard)
            rate.sleep()


if __name__ == "__main__":
    bt = BehaviorTreeManager()
    bt.run()
