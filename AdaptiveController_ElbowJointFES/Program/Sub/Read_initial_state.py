import opensim as osim

def set_initial_state_from_sto(osimmodel, sto_file_path):
        storage = osim.Storage(sto_file_path)
        osimmodel.getSimbodyEngine().convertDegreesToRadians(storage)
        last_time = storage.getLastTime()
        state = osimmodel.initSystem()
        state.setTime(last_time)
        state_vector = storage.getLastStateVector()
        state_vector = state_vector.getData()
        for i in range(state_vector.size()):
            state.updY()[i] = state_vector.get(i)
        osimmodel.equilibrateMuscles(state)
        osimmodel.setPropertiesFromState(state)
        return state