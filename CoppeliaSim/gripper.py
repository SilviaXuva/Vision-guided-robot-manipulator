class RobotiqGripper:
    def __init__(self, client, sim):
        print('Init Gripper...')
        self.client = client
        self.sim = sim
        self.simIK = self.client.getObject('simIK')

        self.j1 = self.sim.getObject('./active1')
        self.j2 = self.sim.getObject('./active2')
        
        self.ik_env = self.simIK.createEnvironment()
        sim_base = self.sim.getObject('./ROBOTIQ85')
        
        self.ik_group1 = self.simIK.createIkGroup(self.ik_env)
        sim_tip1 = self.sim.getObject('./LclosureDummyA')
        sim_target1 = self.sim.getObject('./LclosureDummyB')
        self.simIK.addIkElementFromScene(self.ik_env, self.ik_group1, sim_base, sim_tip1, sim_target1, self.simIK.constraint_x + self.simIK.constraint_z)
        
        self.ik_group2 = self.simIK.createIkGroup(self.ik_env)
        sim_tip2 = self.sim.getObject('./RclosureDummyA')
        sim_target2 = self.sim.getObject('./RclosureDummyB')
        self.simIK.addIkElementFromScene(self.ik_env, self.ik_group2, sim_base, sim_tip2, sim_target2, self.simIK.constraint_x + self.simIK.constraint_z)

        self.connector = self.sim.getObject('./attachPoint')
        self.object_sensor = self.sim.getObject('./attachProxSensor')
    
    def setActuationType(self, close, shape_path = './Cuboid'):
        self.close = close
        shape = self.sim.getObject(shape_path)
        if self.close:
            if self.sim.checkProximitySensor(self.object_sensor, shape)[0] == 1:
                self.sim.setObjectParent(shape, self.connector, True)
        else:
            self.sim.setObjectParent(shape, -1, True)
    
    def actuation(self):
        p1 = self.sim.getJointPosition(self.j1)
        p2 = self.sim.getJointPosition(self.j2)
        if (self.close):
            if (p1<p2-0.008):
                self.sim.setJointTargetVelocity(self.j1, -0.01)
                self.sim.setJointTargetVelocity(self.j2, -0.04)
            else:
                self.sim.setJointTargetVelocity(self.j1, -0.04)
                self.sim.setJointTargetVelocity(self.j2, -0.04)
        else:
            if (p1<p2):
                self.sim.setJointTargetVelocity(self.j1, 0.04)
                self.sim.setJointTargetVelocity(self.j2, 0.02)
            else:
                self.sim.setJointTargetVelocity(self.j1, 0.02)
                self.sim.setJointTargetVelocity(self.j2, 0.04)                
        
        self.simIK.applyIkEnvironmentToScene(self.ik_env, self.ik_group1)
        self.simIK.applyIkEnvironmentToScene(self.ik_env, self.ik_group2)
            
class GripperChildScript:
    def __init__(self, client, sim, gripper_name = './ROBOTIQ85'):
        print('Init Gripper...')
        self.client = client
        self.sim = sim
        
        handle = self.sim.getObject(gripper_name)
        self.script_handle = self.sim.getScript(self.sim.scripttype_childscript, handle)
        self.connector = self.sim.getObject('./attachPoint')
        self.object_sensor = self.sim.getObject('./attachProxSensor')
    
    def actuation(self, close):
        self.sim.callScriptFunction('Actuation', self.script_handle, close)
        
    def open(self):
        self.sim.setObjectParent(self.shape, -1, True)
        self.actuation(False)
        self.client.step()
        
    def close(self, shape_name = './Cuboid'):
        self.actuation(True)
        self.shape = self.sim.getObject(shape_name)
        if self.sim.checkProximitySensor(self.object_sensor,self.shape)[0] == 1:
            self.sim.setObjectParent(self.shape, self.connector, True)
        self.client.step()
