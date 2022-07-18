%% OpenSim Definition
import org.opensim.modeling.*;

% Path to the body parts in .stl format
path = 'C:/Users/Mirevi/Documents/KIKU/IMA/BodyPartSegmentation/BodyParts';

% Set to 1 in order to plot the coordinate frame
plot_co = 0;
axis_len = 1; % length of an axis
thickness = 0.1; % thickness of an axis

% Volume of the current mesh calculated using a surface integral
mass_per_vol_unit = 1000; % mass per volume unit

hip_root = 1; % is the hip the root joint

% Available joints
joints = { 'HIPS', 'SPINE', 'CHEST', 'NECK', 'SHOULDER_L', 'ARM_L', 'FOREARM_L', 'HAND_L', 'SHOULDER_R', 'ARM_R', 'FOREARM_R', 'HAND_R', 'UPLEG_L', 'LEG_L', 'FOOT_L', 'UPLEG_R', 'LEG_R', 'FOOT_R', 'HEAD' };

% child 2 parent: the last column indicates the joint type (W: Weld, B: Ball, P: Pin)
c2p = [
    "FOOT_L", "LEG_L", "B";
    "LEG_L", "UPLEG_L", "B";
    "UPLEG_L", "HIPS", "B";
    "FOOT_R", "LEG_R", "B";
    "LEG_R", "UPLEG_R", "B";
    "UPLEG_R", "HIPS", "B";
    "HAND_L", "FOREARM_L", "B";
    "FOREARM_L", "ARM_L", "B";
    "ARM_L", "SHOULDER_L", "B";
    "SHOULDER_L", "NECK", "B";
    "HAND_R", "FOREARM_R", "B";
    "FOREARM_R", "ARM_R", "B";
    "ARM_R", "SHOULDER_R", "B";
    "SHOULDER_R", "NECK", "B";
    "HEAD", "NECK", "B";
    "NECK", "CHEST", "B";
    "CHEST", "SPINE", "B"; 
    "SPINE", "HIPS", "B";
    "HIPS", "GROUND", "W";
  ];

if(hip_root == 0)
    c2p = [
        "FOOT_L", "GROUND", "W";
        "LEG_L", "FOOT_L", "B";
        "UPLEG_L", "LEG_L", "P";
        "HIPS", "UPLEG_L", "B";
        "FOOT_R", "LEG_R", "B";
        "LEG_R", "UPLEG_R", "P";
        "UPLEG_R", "HIPS", "B";
        "HAND_L", "FOREARM_L", "B";
        "FOREARM_L", "ARM_L", "P";
        "ARM_L", "SHOULDER_L", "B";
        "SHOULDER_L", "NECK", "B";
        "HAND_R", "FOREARM_R", "B";
        "FOREARM_R", "ARM_R", "P";
        "ARM_R", "SHOULDER_R", "B";
        "SHOULDER_R", "NECK", "B";
        "HEAD", "NECK", "B";
        "NECK", "CHEST", "B";
        "CHEST", "SPINE", "B"; 
        "SPINE", "HIPS", "B"
      ];
end


% Get the number of joints
n_joints = numel(joints);

% Instantiate the OpenSim model
model = Model();

% Cell arrays that are used to create a dictionary (Key: Name of a joint, Value: OpenSimb body)
bJoints = cell(1, n_joints);
bodyParts = cell(1, n_joints);

% create OpenSim bodies for all joints
j = 1;
for i = 1:n_joints
    % load the inertia data
    data = mass_per_vol_unit*h5read(strcat(path, "/props.h5"), strcat('/', joints{i}, '_Inertia'));
    % inertia tensor (Vec6) as [Ixx Iyy Izz Ixy Ixz Iyz]
    inertia =Inertia(data(1,1), data(2,2), data(3,3), data(1,2), data(1,3), data(2, 3));
    % load the mass center
    data = h5read(strcat(path, "/props.h5"), strcat('/', joints{i}, '_MassCenter'));
    mass_center = Vec3(data(1), data(2), data(3));
    % TODO assign a appropriate mass to a body
    data = h5read(strcat(path, "/props.h5"), strcat('/', joints{i}, '_Volume'));
    mass = mass_per_vol_unit * data(1);
    % create the OpenSim body
    joint = Body(joints{i}, mass, mass_center, inertia);
    % Load the mesh
    % Path to a joint in .stl format
    fdir = strcat(path, '/', joints{i}, '.stl');
    geom = Mesh(fdir);
    % uncomment to highlight the left hand in red
    %{
    if(strcmp(joints{i}, "HAND_L"))
        geom.setColor(Vec3(1, 0, 0));
    end
    %}
    joint.attachGeometry(geom);
    %  Add body to the OpenSim model
    model.addBody(joint);
    % Store the joint name and the body for later use
    bodyParts{i} = joint;
    bJoints{i} = joints{i};
end

if(plot_co)    
    x = Body('x', 1.0, Vec3(0), Inertia(0));
    ex = Ellipsoid(axis_len,thickness,thickness);
    ex.setColor(Vec3(1, 0, 0));
    x.attachGeometry(ex);
    model.addBody(x);
    
    y = Body('y', 1.0, Vec3(0), Inertia(0));
    ey = Ellipsoid(thickness,axis_len,thickness);
    ey.setColor(Vec3(0, 1, 0));
    y.attachGeometry(ey);
    model.addBody(y);
    
    z = Body('z', 1.0, Vec3(0), Inertia(0));
    ez = Ellipsoid(thickness,thickness,axis_len);
    ez.setColor(Vec3(0, 0, 1));
    z.attachGeometry(ez);
    model.addBody(z);
end

% Dictionary (Key: Name of a joint, Value: OpenSimb body)
bp2joint = containers.Map(bJoints, bodyParts);
% Number of joints that will be created in the OpenSim model
n_joints = size(c2p, 1);

axis = ["RX", "RY", "RZ", "TX", "TY", "TZ"];
ranges = [pi, pi, pi, 10, 10, 10];
n_axis = size(axis, 2);

for i = 1:n_joints
    % Name of the joint
    name = c2p(i, 1);
    % Relative position of the child joint w.r.t. the parent coordinate frame
    locationChildInParent = Vec3(0, 0, 0);
    if(isKey(bp2joint, c2p(i, 2))) % Check, if we have a body of the parent joint (if no: than the parent joint is the ground)
        % get the position of the child
        pos_c = h5read(strcat(path, "/props.h5"), strcat('/', c2p(i, 1), '_Position'));
        % get the position of the parent
        pos_p = h5read(strcat(path, "/props.h5"), strcat('/', c2p(i, 2), '_Position'));
        pivot_c = h5read(strcat(path, "/props.h5"), strcat('/', c2p(i, 1), '_Pivot'));
        
        pos_pivot = pivot_c - pos_c;
        locationInChild = Vec3(pos_pivot(1), pos_pivot(2), pos_pivot(3));
        pos_pc = pos_c - pos_p + pos_pivot;
        locationChildInParent = Vec3(pos_pc(1), pos_pc(2), pos_pc(3));
    end
    % extract the child OpenSim body
    childBody = bp2joint(c2p(i,1));
    % We do not change any rotation of the joints as the body scan is in
    % T-Pose and is considered as zero deviation of the joints
    %isWeld = 0;
    if (strcmp(c2p(i, 3), "W"))
        %isWeld = 1;
        % Root joint to the ground
        %mjoint = WeldJoint(name, model.getGround(), Vec3(0,0,0), Vec3(0,0,0), childBody, Vec3(0,0,0), Vec3(0,0,0));
        mjoint = FreeJoint(name, model.getGround(), Vec3(0,0,0), Vec3(0,0,0), childBody, Vec3(0,0,0), Vec3(0,0,0));
        axis = ["RX", "RY", "RZ", "TX", "TY", "TZ"];
        ranges = [pi, pi, pi, 10, 10, 10];
        n_axis = size(axis, 2);
    elseif(strcmp(c2p(i, 3), "B"))
        % extract the parent OpenSim body
        parentBody = bp2joint(c2p(i,2));
        mjoint = BallJoint(name, parentBody, locationChildInParent, Vec3(0,0,0), childBody, locationInChild, Vec3(0,0,0));
        axis = ["RX", "RY", "RZ"];
        ranges = [pi, pi, pi];
        n_axis = size(axis, 2);
    elseif(strcmp(c2p(i, 3), "P"))
        parentBody = bp2joint(c2p(i,2));
        mjoint = PinJoint(name, parentBody, locationChildInParent, Vec3(0,0,0), childBody, locationInChild, Vec3(0,0,0));
        axis = ["RZ"];
        ranges = [pi];
        n_axis = size(axis, 2);
    else
        continue
    end
    for j = 1:n_axis
        axis_name = strcat(name, "_", axis(j));
        j_axis = mjoint.upd_coordinates(j-1);
        j_axis.setName(axis_name);
        j_axis.setRange([-ranges(j), ranges(j)]);
        j_axis.setDefaultValue(0);
    end
    % Add the joint to the model
    model.addJoint(mjoint);
end

if (plot_co)
    ground_x = WeldJoint('ground_x', model.getGround(), Vec3(axis_len/2,0,0), Vec3(0,0,0), x, Vec3(0,0,0), Vec3(0,0,0));
    model.addJoint(ground_x);
    ground_y = WeldJoint('ground_y', model.getGround(), Vec3(0,axis_len/2,0), Vec3(0,0,0), y, Vec3(0,0,0), Vec3(0,0,0));
    model.addJoint(ground_y);
    ground_z = WeldJoint('ground_z', model.getGround(), Vec3(0,0,axis_len/2), Vec3(0,0,0), z, Vec3(0,0,0), Vec3(0,0,0));
    model.addJoint(ground_z);
end

model.finalizeConnections();
model.print('kiku_model.osim');


% Simulate the model
%{
model.setUseVisualizer(true);
initState = model.initSystem();
disp(initState.getY());
finalState = opensimSimulation.simulate(model, initState, 0.01);
%}