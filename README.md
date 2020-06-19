# Teaching an Actor-Critic Reinforcment Learning Agent to Land a Rocket in KSP

## env.py
* Contains telemetry streaming code and interface to control a rocket in KSP
* Observation State value function needs work
    * What factors should contribute to value of a given state?
        * north, east distance to target (lat,long)?
        * touchdown vertical,horizontal velocity
        * altitude?
        * number of engine ignitions?
        * g-force minimization?
        * fuel usage?
    