## Note:
Feedback of the previous report

## Content:
Introduction: Very clear detailed and well thought out rationale for the algorithms and data types used. 

Methods no formalisation of the checks is presented (-4pts). Step 6 is doing most of your heavy lifting, it would be clearer if you broke this module down into sub-components and explained in more detail each check and the order it is applied. Similarly, I assume by 'in parallel' you mean you consider the trajectory list in parallel and then each criteria in Step 6 is sequentially applied but this is not explicitly stated. "Content df" appears to be a typo, not sure what df is but it does not appear to be defined. 

Validation There was no check for the length of the trajectory (-1pts), however the intersection and angle tests were present. As a note (no points taken off) it is good practice to have functions with computations typically return a value that is not zero -- so for the angle test it would have been a slightly better practice to adjust the line so that you have a 45 degree return, this is just because it is too common to have null or invalid computations return 0 as a default value and hence cause a silent failure.

Code: Worked with a clear readme to understand installation. Length of trajectory test not implemented (-1pts). Otherwise code was clear, well documented and in line with the description in the methodology.