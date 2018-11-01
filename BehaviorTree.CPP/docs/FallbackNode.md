# Fallback

This family of nodes are known as "Selector" or, sometimes, "Priority"
in other frameworks.

Its purpose is to try different strategies, until we find one that "works".

Currently the framework provides two kinds of nodes:

- FallbackNode
- FallbackStarNode

They share the following rules:

- Before ticking the first child, the node status becomes __RUNNING__.

- If a child returns __FAILURE__, it ticks the next child.

- If the __last__ child returns __FAILURE__ too, all the children are halted and
 the sequence returns __FAILURE__.
 
- If a child returns __SUCCESS__, it stops and returns __SUCCESS__.
  All the children are halted. 


## FallbackNode

If a child returns __RUNNING__:

- FallbackNode returns __RUNNING__. 
- The loop is restarted and  all the previous children are ticked again __unless
  they are ActionNodes__. 


__Example__:

Try different strategies to open the door. Check first if the door is open.

![FallbackNode](images/FallbackSimplified.png)

??? example "See the pseudocode"
	``` c++
		status = RUNNING;

		for (int index=0; index < number_of_children; index++)
		{
			child_status = child[index]->tick();
			
			if( child_status == RUNNING ) {
				// Suspend execution and return RUNNING.
				// At the next tick, index will be the same.
				return RUNNING;
			}
			else if( child_status == SUCCESS ) {
				// Suspend execution and return SUCCESS.
				// index is reset and children are halted.
				HaltAllChildren();
				return SUCCESS;
			}
		}
		// all the children returned FAILURE. Return FAILURE too.
		HaltAllChildren();
		return FAILURE;
	```	

## FallbackStarNode

If a child returns __RUNNING__:

- FallbackStarNode returns __RUNNING__. 
- The loop is __not__ restarted and  none of the previous children is ticked.

??? example "See the pseudocode"
	``` c++
		// index is initialized to 0 in the constructor
		status = RUNNING;

		while( index < number_of_children )
		{
			child_status = child[index]->tick();
			
			if( child_status == RUNNING ) {
				// Suspend execution and return RUNNING.
				// At the next tick, index will be the same.
				return RUNNING;
			}
			else if( child_status == FAILURE ) {
				// continue the while loop
				index++;
			}
			else if( child_status == SUCCESS ) {
				// Suspend execution and return SUCCESS.
				// At the next tick, index will be the same.
   			    HaltAllChildren();
				index = 0;
				return SUCCESS;
			}
		}
		// all the children returned FAILURE. Return FAILURE too.
		index = 0;
		HaltAllChildren();
		return FAILURE;
	```	


 
