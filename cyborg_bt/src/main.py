#!/usr/bin/env python

import b3
import json
import pydot as pd
import rospy
from std_msgs.msg import String

from cyborg_bt_nodes.actions import MoveTo


NAME = 'behavior_tree_manager'


class BehaviorTreeManager():
    def __init__(self):
        rospy.init_node(NAME)

        rospy.loginfo('Initializing: %s' % NAME)

        target = None
        blackboard = b3.Blackboard()
        names = {'MoveTo': MoveTo}

        tree_file = rospy.get_param('~tree_file')
        rospy.loginfo('Loading project from %s' % tree_file)
        with open(tree_file) as f:
            data = json.load(f)

        trees = self.load_project(data, names)
        root_tree = trees[data['selectedTree']]

        rospy.loginfo('Running %s' % root_tree)

        if root_tree is not None:
            self.run(root_tree, target, blackboard)

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

        return trees

    def create_graph(self, tree):
        """
        Generate a pydot graph.

        Args:
            tree: the behavior tree
            level: collapses trees at or below this level

        Returns:
            pydot.Dot: graph
        """
        def process_node(graph, node):
            if isinstance(node, b3.Composite):
                for c in node.children:
                    if isinstance(c, b3.BehaviorTree):
                        subgraph = pd.Subgraph(c.title)
                        graph.add_subgraph(subgraph)
                        graph.add_edge(node.graph_edge(c.root))
                        process_tree(subgraph, c)
                    else:
                        graph.add_node(c.graph_node)
                        graph.add_edge(node.graph_edge(c))
                        process_node(graph, c)

        def process_tree(graph, tree):
            root = tree.root
            graph.set_name = tree.title
            graph.add_node(root.graph_node)
            process_node(graph, root)

        graph = pd.Dot(graph_type='digraph')
        process_tree(graph, tree)

        return graph

    def run(self, tree, target, blackboard):
        pub = rospy.Publisher('cyborg_bt/tree', String, queue_size=10)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            tree.tick(target, blackboard)
            G = self.create_graph(tree)
            pub.publish(G.to_string())
            rate.sleep()


if __name__ == "__main__":
    rospy.set_param('~tree_file', '/home/mortenmj/.ros/project.json')
    BehaviorTreeManager()
