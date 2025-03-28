"""
Inputs: 
    Array of timestamps
    Array holding hand data values
"""

import numpy as np

# Sample input data (replace with actual values)
tIn = np.array([0.0, 0.5, 1.0, 1.5, 2.0])  # Timestamps
hIn = np.array([1.2, 2.3, 3.1, 2.8, 1.5])  # Hand data values

# Initialize parameters
kMax = len(tIn)
tMax = tIn[-1]
tMin = tIn[0]
tBase = (tMax - tMin) / kMax

# Compute baseSet
baseSet = np.array([tBase * (k + 1) for k in range(kMax)])

# Expectation step: Compute sum of h(t - tk)
def expectX(t,tSet,handData):
    '''
    Computes expected x(t) for one value of t in tSet

    t - tk will not always be a value in tSet 
    Need to match t - tk with the correct value in hIn
    '''
    xT = 0
    for tk in tSet:
        t - tk
        
        
    return np.array([np.sum(hIn[t - tk]) for tk in tSet])

E = np.array([])
for t in baseSet:
    print(expectX(t,baseSet,hIn))
    E = np.append(E,expectX(t,baseSet,hIn))
print("Expectation values:", E)

# Maximization