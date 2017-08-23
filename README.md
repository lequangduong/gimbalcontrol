# Model and Control three-axis Gimbal using brushless motor with drone parameter input

Step 1: Construct a mathematical model of three-axis Gimbal.

  The goal is build a three axis gimbal hanging under drone which have precisely position and direction using to tracking a object           freely move on the the ground. The inverse kinematics problem was solved from input parameters include position of object and             position and direction of drone (or the base link of Gimbal).The results are angles of three links of Gimbal and their value was           entered into Simulink Model that had built using Simscape Multibody toolbox in Matlab.
  
  [Link video simulation here](https://youtu.be/cXD1rDbsp4I)
        
Step 2: Implement and compare LQR/LQG and MPC control algorithms for the three-axis Gimbal Model

    Outcome with MPC constraint: -6 < u < 6 (kN) 
  
  The video simulation:
  
  [LQG here](https://youtu.be/W8o5ViPu-cI)

  [MPC here](https://youtu.be/HDwamu4LJnE)

![1](https://user-images.githubusercontent.com/12315370/29620279-ae4891be-8847-11e7-8632-5261d5d39bed.jpg)
![2](https://user-images.githubusercontent.com/12315370/29620277-ae45fabc-8847-11e7-90e0-5e80933a2a45.jpg)
![3](https://user-images.githubusercontent.com/12315370/29620278-ae48312e-8847-11e7-8d95-407ec9f3b156.jpg)
![4](https://user-images.githubusercontent.com/12315370/29620274-ae016db6-8847-11e7-96aa-dc3c93763565.jpg)
![5](https://user-images.githubusercontent.com/12315370/29620273-adfbffac-8847-11e7-9352-b74264fa5d4e.jpg)
![6](https://user-images.githubusercontent.com/12315370/29620280-aed9ebbe-8847-11e7-8dcf-ec129500abd0.jpg)
![7](https://user-images.githubusercontent.com/12315370/29620272-adfbf192-8847-11e7-9def-8ff399e36882.jpg)
![8](https://user-images.githubusercontent.com/12315370/29620276-ae1933ec-8847-11e7-873a-e1acefb421b2.jpg)
![9](https://user-images.githubusercontent.com/12315370/29620275-ae05a91c-8847-11e7-9fd0-ac8b82d3f6e2.jpg)
