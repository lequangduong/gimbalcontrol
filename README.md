# Model and Control three-axis Gimbal using brushless motor with drone parameter input

Step 1: Construct a mathematical model of three-axis Gimbal.

  The goal is build a three axis gimbal hanging under drone which have precisely position and direction using to tracking a object           freely move on the the ground. The inverse kinematics problem was solved from input parameters include position of object and             position and direction of drone (or the base link of Gimbal).The results are angles of three links of Gimbal and their value was           entered into Simulink Model that had built using Simscape Multibody toolbox in Matlab.
  
  [Link video simulation here](https://youtu.be/cXD1rDbsp4I)
        
Step 2: Implement and compare LQR/LQG and MPC control algorithms for the three-axis Gimbal Model

    Outcome with MPC constraint: -6 < u < 6 (kN) 
  
  The video simulation:
  
  [LQG here](https://youtu.be/W8o5ViPu-cI)

  [MPC here](https://youtu.be/HDwamu4LJnE)

![1](https://user-images.githubusercontent.com/12315370/29611271-22118e8c-8827-11e7-82a6-7eb7787a9847.jpg)
![2](https://user-images.githubusercontent.com/12315370/29611268-21fc8e74-8827-11e7-80ca-cbf95f58dd93.jpg)
![3](https://user-images.githubusercontent.com/12315370/29611272-2217ec0a-8827-11e7-9970-f9a7e52259e9.jpg)
![4](https://user-images.githubusercontent.com/12315370/29611273-2239c096-8827-11e7-8815-6f915780b45c.jpg)
![5](https://user-images.githubusercontent.com/12315370/29611275-2242a6c0-8827-11e7-8c8f-fa298ffa8c7f.jpg)
![6](https://user-images.githubusercontent.com/12315370/29611274-2240ae06-8827-11e7-9c94-86721e59c489.jpg)
![7](https://user-images.githubusercontent.com/12315370/29611267-21f4b56e-8827-11e7-8cf9-98936001be2b.jpg)
![8](https://user-images.githubusercontent.com/12315370/29611269-220ca64c-8827-11e7-8522-d9fdd7d51dbc.jpg)
![9](https://user-images.githubusercontent.com/12315370/29611270-220f1648-8827-11e7-826f-35d7904e0043.jpg)
