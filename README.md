# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflection

This PID controller project, although easy to code, provided quite the challenge when it caming to tuning Hyperparameters. I really had to push myself to be less of a perfectionist when it came to tuning how the vehicle controlled itself.

After multiple iterations of hand tuning parameters, I decided to implement the Twiddle algorithm for tuning PID gains. The challenge came in coming up with an implementation that fit into the existing webSocket scheme. I had to implement an interactive Twiddle algorithm that allowed the program to run unimpeded while keeping track of certain process states to determine how to run the Twiddle algorithm. This step is what took the majority of my time with this project.

The end result was a a large switch statement that used process tracking variables to select the right case, given previous cost function results.

After implementing the 
