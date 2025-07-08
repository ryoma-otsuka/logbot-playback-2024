# https://gist.github.com/eherrerosj/ed3400b06f3c5c7668c62653e2a695c2

import numpy as np
from sklearn.tree import _tree

def tree_to_code_py(tree, feature_names):
    tree_ = tree.tree_
    feature_name = [
        feature_names[i] if i != _tree.TREE_UNDEFINED else "undefined!"
        for i in tree_.feature
    ]
    print ("def tree({}):".format(", ".join(feature_names)))

    def recurse(node, depth):
        indent = "    " * depth
        if tree_.feature[node] != _tree.TREE_UNDEFINED:
            name = feature_name[node]
            threshold = tree_.threshold[node]
            print ("{}if {} <= {}:".format(indent, name, threshold))
            recurse(tree_.children_left[node], depth + 1)
            print ("{}else:  # if {} > {}".format(indent, name, threshold))
            recurse(tree_.children_right[node], depth + 1)
        else:
            print ("{}return {}".format(indent, np.argmax(tree_.value[node])))

    recurse(0, 1)


def tree_to_code_py_02(tree, feature_names, tree_name, target_list):
    """
    Outputs a decision tree model as a cpp function

    Parameters:
    -----------
    tree: decision tree model
        The decision tree to represent as a function
    feature_names: list
        The feature names of the dataset used for building the decision tree
    """
    
    with open(tree_name + ".py", "w") as code_file:
        tree_ = tree.tree_
        feature_name = [feature_names[i] if i != _tree.TREE_UNDEFINED else "undefined!" for i in tree_.feature]
        code_file.write("def tree({}):\n".format(", ".join(feature_names)))
        # print ("def tree({}):".format(", ".join(feature_names)))
        
        def recurse(node, depth):
            indent = "    " * depth
            if tree_.feature[node] != _tree.TREE_UNDEFINED:
                name = feature_name[node]
                threshold = tree_.threshold[node]
                code_file.write("{}if {} <= {}:\n".format(indent, name, threshold))
                # print ("{}if {} <= {}:".format(indent, name, threshold))
                recurse(tree_.children_left[node], depth + 1)
                code_file.write("{}else:  # if {} > {}\n".format(indent, name, threshold))
                # print ("{}else:  # if {} > {}".format(indent, name, threshold))
                recurse(tree_.children_right[node], depth + 1)
            else:
                class_values = tree_.value[node][0]
                max_idx = 0
                for i in range(len(class_values)):
                    if class_values[i] > class_values[max_idx]:
                        max_idx = i
                code_file.write("{}return {}".format(indent, np.argmax(tree_.value[node])))
                code_file.write(" # " + target_list[max_idx] + "\n")
                # print ("{}return {}".format(indent, np.argmax(tree_.value[node])))

        recurse(0, 1)



def tree_to_code_cpp(tree, feature_names, tree_name, target_list):
    """
    Outputs a decision tree model as a cpp function

    Parameters:
    -----------
    tree: decision tree model
        The decision tree to represent as a function
    feature_names: list
        The feature names of the dataset used for building the decision tree
    """

    with open(tree_name + ".cpp", "w") as code_file:
        tree_ = tree.tree_
        feature_name = [feature_names[i] if i != _tree.TREE_UNDEFINED else "undefined!" for i in tree_.feature]
        code_file.write("static uint8_t tree()\n{\n")
        #print("uint8_t tree()\n{")

        def recurse(node, depth):
            indent = "  " * depth
            if tree_.feature[node] != _tree.TREE_UNDEFINED:
                name = feature_name[node]
                threshold = tree_.threshold[node]
                code_file.write(indent + "if (" + name + " <= " + str(threshold) + ")\n")
                code_file.write(indent + "{\n")
                #print(indent + "if (" + name + " <= " + str(threshold) + ")")
                #print(indent + "{")
                recurse(tree_.children_left[node], depth + 1)
                code_file.write(indent + "}\n")
                code_file.write(indent + "else\n")
                code_file.write(indent + "{\n")
                #print(indent + "}")
                #print(indent + "else")
                #print(indent + "{")
                recurse(tree_.children_right[node], depth + 1)
                code_file.write(indent + "}\n")
                #print(indent + "}")
            else:
                class_values = tree_.value[node][0]
                max_idx = 0
                for i in range(len(class_values)):
                    if class_values[i] > class_values[max_idx]:
                        max_idx = i
                code_file.write(indent + "return " + str(max_idx) + "; // " + target_list[max_idx] + "\n")
                # if tree_.value[node][0][0] < tree_.value[node][0][1]:
                #     code_file.write(indent + "return true;\n")
                #     #print(indent + "return true;")
                # else:
                #     code_file.write(indent + "return false;\n")
                #     #print(indent + "return false;")

        recurse(0, 1)
        code_file.write("}\n")
        #print("}")
