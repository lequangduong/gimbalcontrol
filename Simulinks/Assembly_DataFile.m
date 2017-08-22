% Simscape(TM) Multibody(TM) version: 5.0

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(10).translation = [0.0 0.0 0.0];
smiData.RigidTransform(10).angle = 0.0;
smiData.RigidTransform(10).axis = [0.0 0.0 0.0];
smiData.RigidTransform(10).ID = '';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [-10.000000000000009 0 0];  % mm
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [0.57735026918962584 0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(1).ID = 'B[Link1-1:-:Link2-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-135.00000000000017 71.999999999999758 4.5474735088646412e-13];  % mm
smiData.RigidTransform(2).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(2).axis = [0.57735026918962573 0.57735026918962562 0.57735026918962573];
smiData.RigidTransform(2).ID = 'F[Link1-1:-:Link2-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0 0 9.9999999999997868];  % mm
smiData.RigidTransform(3).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(3).axis = [1 0 0];
smiData.RigidTransform(3).ID = 'B[Base-1:-:Link1-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [129.99999999999994 3.1226688114278474e-14 -145];  % mm
smiData.RigidTransform(4).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(4).axis = [1 5.9947801016780365e-48 5.078408089523213e-32];
smiData.RigidTransform(4).ID = 'F[Base-1:-:Link1-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [0 0 0];  % mm
smiData.RigidTransform(5).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(5).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(5).ID = 'B[Link2-1:-:Assem1^Assembly-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-32.499999999999851 -48.999999999999957 -14.047898896116756];  % mm
smiData.RigidTransform(6).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(6).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(6).ID = 'F[Link2-1:-:Assem1^Assembly-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [0 0 0];  % mm
smiData.RigidTransform(7).angle = 0;  % rad
smiData.RigidTransform(7).axis = [0 0 0];
smiData.RigidTransform(7).ID = 'B[Base-1:-:]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [0 0 4000];  % mm
smiData.RigidTransform(8).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(8).axis = [1 -2.0816681711721685e-16 -0];
smiData.RigidTransform(8).ID = 'F[Base-1:-:]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [0 0 0];  % mm
smiData.RigidTransform(9).angle = 0;  % rad
smiData.RigidTransform(9).axis = [0 0 0];
smiData.RigidTransform(9).ID = 'AssemblyGround[Assem1^Assembly-1:Tool-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [-32.5 -40.999999999999964 -14.047898896115907];  % mm
smiData.RigidTransform(10).angle = 0;  % rad
smiData.RigidTransform(10).axis = [0 0 0];
smiData.RigidTransform(10).ID = 'AssemblyGround[Assem1^Assembly-1:Link3-1]';


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(5).mass = 0.0;
smiData.Solid(5).CoM = [0.0 0.0 0.0];
smiData.Solid(5).MoI = [0.0 0.0 0.0];
smiData.Solid(5).PoI = [0.0 0.0 0.0];
smiData.Solid(5).color = [0.0 0.0 0.0];
smiData.Solid(5).opacity = 0.0;
smiData.Solid(5).ID = '';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.32324748295221495;  % kg
smiData.Solid(1).CoM = [-57.91435451083786 52.606673625227714 0];  % mm
smiData.Solid(1).MoI = [1289.0474396076411 817.58846837763463 2076.2750412027613];  % kg*mm^2
smiData.Solid(1).PoI = [0 0 363.05605012408364];  % kg*mm^2
smiData.Solid(1).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = 'Link2*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.53139285984774642;  % kg
smiData.Solid(2).CoM = [-32.340826926174834 23.574916252866192 -0.088245413679890569];  % mm
smiData.Solid(2).MoI = [1058.4103278708226 433.95475880821016 807.52391932293233];  % kg*mm^2
smiData.Solid(2).PoI = [1.8018667890836746 -0.26537550529511039 3.9222405585727409];  % kg*mm^2
smiData.Solid(2).color = [1 1 1];
smiData.Solid(2).opacity = 0.31999999999999995;
smiData.Solid(2).ID = 'Tool*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.13868726345575022;  % kg
smiData.Solid(3).CoM = [0.017646526985192111 61.51561153632322 42.317932062948636];  % mm
smiData.Solid(3).MoI = [534.26584078318365 92.592872924362851 466.34774982703254];  % kg*mm^2
smiData.Solid(3).PoI = [-14.752931001770573 -0.04706243119713141 0.049393235167847056];  % kg*mm^2
smiData.Solid(3).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = 'Link3*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.32341219385368764;  % kg
smiData.Solid(4).CoM = [13.248129714370393 -3.194240725386745e-05 -76.417059874285371];  % mm
smiData.Solid(4).MoI = [1396.7094383221877 2011.0768652429326 675.88269210900864];  % kg*mm^2
smiData.Solid(4).PoI = [-0.00061355703553829738 697.45472154544177 0.00061752082817199212];  % kg*mm^2
smiData.Solid(4).color = [0.90980392156862744 0.44313725490196076 0.031372549019607843];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = 'Link1*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.23240584136024758;  % kg
smiData.Solid(5).CoM = [0.083920961848354292 -0.086437177525766962 -14.037074373103589];  % mm
smiData.Solid(5).MoI = [166.7022307926907 166.78954922874632 269.69868734267641];  % kg*mm^2
smiData.Solid(5).PoI = [-0.17000752349703233 0.16505854658516075 0.0013632407559639812];  % kg*mm^2
smiData.Solid(5).color = [0.75294117647058822 0 0];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = 'Base*:*Default';


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(3).Rz.Pos = 0.0;
smiData.RevoluteJoint(3).ID = '';

smiData.RevoluteJoint(1).Rz.Pos = 1.3863716440904667e-14;  % deg
smiData.RevoluteJoint(1).ID = '[Link1-1:-:Link2-1]';

smiData.RevoluteJoint(2).Rz.Pos = 3.6576378836830441e-14;  % deg
smiData.RevoluteJoint(2).ID = '[Base-1:-:Link1-1]';

smiData.RevoluteJoint(3).Rz.Pos = 7.7907360686059021e-15;  % deg
smiData.RevoluteJoint(3).ID = '[Link2-1:-:Assem1^Assembly-1]';

