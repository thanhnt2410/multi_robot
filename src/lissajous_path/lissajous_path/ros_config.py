"""Common scenario parameters for ROS nodes."""
from .core import Scenario


def scenario_from_node(node):
    defaults=dict(A=7.2, B=3.2, a=3, b=2, omega=0.075, order=[1,2,5,3,4], p=2)
    values={k: node.declare_parameter(k,v).value for k,v in defaults.items()}
    values['order']=tuple(values['order'])
    return Scenario(**values)
