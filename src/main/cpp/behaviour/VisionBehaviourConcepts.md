# Vision Behaviours
## Align
1. pass in a target
2. gets pose (using pose estimator)
3. gos to target (using go to pose behaviour)
## Identify cones and cubes
1. pass in a list of colours it wants to collect
2. identify which targets it can see are within those colours
3. change target list to the list with only those colours
## Select best cone or cube
1. get list of photon tracked targets
2. take out best target
3. get area of target
4. if area == cone or cube      | else
5. select target                | remove target from list
6.                              | get new list
7.                              | repeat from step 2