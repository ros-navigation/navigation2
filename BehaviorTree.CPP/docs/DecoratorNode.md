# Decorators

A decorator is a node that can have only a single child.

It is up to the Decorator to decide if, when and how many times the child should be
ticked.

## InverterNode

Tick the child once and return SUCCESS if the child failed or FAILURE if
the child succeeded.

If the child returns RUNNING, this node returns RUNNING too.

## ForceSuccessNode

If the child returns RUNNING, this node returns RUNNING too. 

Otherwise, it returns always SUCCESS.

## ForceFailureNode

If the child returns RUNNING, this node returns RUNNING too. 

Otherwise, it returns always FAILURE.

## RepeatNode

Tick the child up to N times, where N is passed as a [NodeParameter](tutorial_B_node_parameters.md),
as long as the child returns SUCCESS.

Interrupt the loop if the child returns FAILURE and, in that case, return FAILURE too.

If the child returns RUNNING, this node returns RUNNING too.

## RetryNode

Tick the child up to N times, where N is passed as a [NodeParameter](tutorial_B_node_parameters.md),
as long as the child returns FAILURE.

Interrupt the loop if the child returns SUCCESS and, in that case, return SUCCESS too.

If the child returns RUNNING, this node returns RUNNING too.


