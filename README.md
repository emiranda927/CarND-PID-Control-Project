# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflection

This PID controller project, although relatively easy to code, provided quite the challenge when it came to tuning Hyperparameters. I really had to push myself to be less of a perfectionist when it came to how the vehicle controlled itself on the track

After multiple iterations of hand tuning parameters, I decided to implement the Twiddle algorithm for more finely tuning PID gains. The real challenge was in coming up with an implementation that fit into the existing webSocket scheme. I had to implement an interactive Twiddle algorithm that allowed the program to run unimpeded while keeping track of certain process states to determine where in the process flow the Twiddle algorithm needed to be. This step is what took the majority of my time with this project.

The end result was a very large switch statement that used process tracking and parameter tracking variables to select the right case, given previous cost function results.

After implementing these changes, I was able to to get the vehicle to drive around the track safely, with a moderate speed. However, I wanted to achieve a much higher speed on the track than the 40-50 mph I was able to achieve without derailing the vehicle.

I thought about implementing a PID controller for the throttle control, however, I decided to take a different (and simpler) approach. Since speed was my goal, I decided to try and emulate the behavior of a race car driver on the track and use the brake to control the vehicle through corners.

I implemented a small algorithm that applied the brakes whenever it detected that the steering angle was diverging past an acceptable threshold. This allowed the vehicle to achieve a much higher top speed (around 75 mph on the straight away), while allowing it to brake, slow down and control itself through turns.

Some challenges with this method were if the vehicle was incorrectly tuned, it would be constantly braking and not go anywhere. This was easily solved with a working Twiddle algorithm. The tuning algorithm wasn't enough to overcome the physics of vehicle dynamics, however. If the vehicle slowed down too much through turns, the turning radius gets much smaller, causing the vehicle to leave the track. This problem pushed me to include a speed threshold in braking. After careful observation, I determined that ~45 mph was the slowest the vehicle could go before I should cut off it's ability to brake. This ensured that the PID controller could still do its job at the speeds it was tuned on.

One thing I was not satisfied with is the overall stability of the vehicle. This is due in large part to the control line changing but also not enough damping on the system. Future iterations of this project will include the ability to safely filter out large steering angle deltas based on the change rate. In the real world, it doesn't make sense for the steering angle rate to be extremely large. That only spells disaster for on-road vehicles! This would ensure for much smoother operation and less sea-sick passengers!

