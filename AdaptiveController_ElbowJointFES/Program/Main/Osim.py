import os

#add openSim dir
os.add_dll_directory("C:/Users/Make/OpenSim 4.5/bin")
import opensim as osim

from AdaptiveController_ElbowJointFES.Program.Sub.Read_initial_state import set_initial_state_from_sto


class OsimModel:
    """
    Call Osim to get formulate a simple arm Muscle_models. This Muscle_models calculates and
    outputs the arms angle in different inputs.
    """
    def __init__(self, model_folder_name, model_file_name,initial_sto_file,
                 excitation_BIC_min,excitation_TRI_min,TIME_STEP):

        self.model_folder = model_folder_name
        self.model_file = model_file_name
        self.osimmodel = osim.Model("{}/{}".format(model_folder_name, model_file_name))

        #self.state = self.osimmodel.initSystem()

        #Get muscle models
        BIClong = self.osimmodel.getMuscles().get("BIClong")
        BICshort = self.osimmodel.getMuscles().get("BICshort")
        TRIlong = self.osimmodel.getMuscles().get("TRIlong")
        TRIlat = self.osimmodel.getMuscles().get("TRIlat")
        TRImed = self.osimmodel.getMuscles().get("TRImed")
        Bra = self.osimmodel.getMuscles().get("BRA") #not used

        #Make controllers of the muscles
        self.controller_dict = {}
        self.__create_controller("BIClong",BIClong, excitation_BIC_min)
        self.__create_controller("BICshort",BICshort, excitation_BIC_min)
        self.__create_controller("TRIlong",TRIlong, excitation_TRI_min)
        self.__create_controller("TRIlat",TRIlat, excitation_TRI_min)
        self.__create_controller("TRImed",TRImed, excitation_TRI_min)
        self.__create_controller("Bra",Bra, excitation_BIC_min)

        # Set CoordinateLimitForce(Elbow joint range of motion,
        # and expression in motion limits)
        elbow_limit_force = osim.CoordinateLimitForce("r_elbow_flex",
                                                      150, 5, 1.5, 5, 0.1, 0.001)
        #LAUNCH THE MODEL AND STATE // INITIALIAZE
        self.osimmodel.addForce(elbow_limit_force)
        self.state = self.osimmodel.initSystem()
        self.state = set_initial_state_from_sto(self.osimmodel, initial_sto_file)

        self.manager = osim.Manager(self.osimmodel)
        self.manager.setIntegratorMethod(osim.Manager.IntegratorMethod_RungeKuttaMerson)
        self.manager.setIntegratorMaximumStepSize(TIME_STEP)
        self.manager.setIntegratorMinimumStepSize(TIME_STEP)
        self.manager.initialize(self.state)
        self.setStateTime(0.0) #start time is 0 s

    def __create_controller(self, name, actuator,begin_excitation):
        # Creates a controller for called muscle and adds it to known dict
        controller = osim.PrescribedController()
        controller.setName(f"controller_{name}")
        excitation_function = osim.Constant(begin_excitation)
        controller.addActuator(actuator)
        controller.prescribeControlForActuator(actuator.getName(),
                                               excitation_function)
        self.controller_dict.update({name : controller})
        self.osimmodel.addController(controller)

    def getObservedAngle(self):
        return self.osimmodel.getStateVariableValues(self.state).get(2)

    def getObservedAngularVelocity(self):
        return self.osimmodel.getStateVariableValues(self.state).get(3)

    def changeExcitationBIC(self, excitation):
        """
        Changes the Arms BICeps excitation state. All muscle work
          simultaneously
        Args:
            excitation: Float, Amount excitated

        """
        excitation_function_BIClong = osim.Constant(excitation)
        excitation_function_BICshort = osim.Constant(excitation)
        self.controller_dict["BIClong"].prescribeControlForActuator("BIClong",
                                                                    excitation_function_BIClong)
        self.controller_dict["BICshort"].prescribeControlForActuator("BICshort",
                                                                     excitation_function_BICshort)

    def changeExcitationTRI(self, excitation):
        """
          Changes the Arms TRIceps excitation state. All muscle work
          simultaneously
          Args:
              excitation: Float, Amount excitated

          """
        excitation_function_TRIlong = osim.Constant(excitation)
        excitation_function_TRIlat = osim.Constant(excitation)
        excitation_function_TRImed = osim.Constant(excitation)

        self.controller_dict["TRIlong"].prescribeControlForActuator("TRIlong",
                                                                    excitation_function_TRIlong)
        self.controller_dict["TRIlat"].prescribeControlForActuator("TRIlat",
                                                                   excitation_function_TRIlat)
        self.controller_dict["TRImed"].prescribeControlForActuator("TRImed",
                                                                   excitation_function_TRImed)

    def setStateTime(self,time):
        self.state.setTime(time)

        # Integrate
    def integrateTime(self,current_time,TIME_STEP):
        self.state = self.manager.integrate(current_time + TIME_STEP)

    def printState(self):
        self.osimmodel.printBasicInfo()
