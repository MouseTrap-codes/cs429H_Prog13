# cs429H_Prog13

pb27243

i tested using the testbench and its several iterations

I optimized the pipleine by 

- identifying both load use hazards and self-dependent operations where rd=rs=rt
- designing forwarding logic to prioritize the most recent values from earlier pipeline stages to efficently manage data dependencies and correctly compute results even with back-to-back register usage
- I implemented multiple stalling conditons based on if we are stalling because of a hazard of because of return, and I had proper pipleine flushing for branches
- my writeback has simple selection logic to writeback the correct values during execution

