// Copyright (c) 2012 CNRS
// Author: Florent Lamiraux, Joseph Mirabel
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include "rbprmbuilder.impl.hh"
#include "hpp/rbprm/rbprm-device.hh"
#include "hpp/rbprm/rbprm-validation.hh"
#include "hpp/rbprm/rbprm-path-interpolation.hh"
#include "hpp/rbprm/stability/stability.hh"
#include "hpp/model/urdf/util.hh"
#include <fstream>

#include <hpp/rbprm/fullbodyBallistic/parabola-library.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-path.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-interpolation.hh>
#include "hpp/rbprm/projection-shooter.hh"


namespace hpp {
  namespace rbprm {
    namespace impl {
      using model::displayConfig;

    RbprmBuilder::RbprmBuilder ()
    : POA_hpp::corbaserver::rbprm::RbprmBuilder()
    , romLoaded_(false)
    , fullBodyLoaded_(false)
    , bindShooter_()
    {
        // NOTHING
    }

    void RbprmBuilder::loadRobotRomModel(const char* robotName,
         const char* rootJointType,
         const char* packageName,
         const char* modelName,
         const char* urdfSuffix,
         const char* srdfSuffix) throw (hpp::Error)
    {
        try
        {
            hpp::model::DevicePtr_t romDevice = model::Device::create (robotName);
            romDevices_.insert(std::make_pair(robotName, romDevice));
            hpp::model::urdf::loadRobotModel (romDevice,
                    std::string (rootJointType),
                    std::string (packageName),
                    std::string (modelName),
                    std::string (urdfSuffix),
                    std::string (srdfSuffix));
        }
        catch (const std::exception& exc)
        {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
        }
        romLoaded_ = true;
    }

    void RbprmBuilder::loadRobotCompleteModel(const char* robotName,
         const char* rootJointType,
         const char* packageName,
         const char* modelName,
         const char* urdfSuffix,
         const char* srdfSuffix) throw (hpp::Error)
    {
        if(!romLoaded_)
        {
            std::string err("Rom must be loaded before loading complete model") ;
            hppDout (error, err );
            throw hpp::Error(err.c_str());
        }
        try
        {
            hpp::model::RbPrmDevicePtr_t device = hpp::model::RbPrmDevice::create (robotName, romDevices_);
            hpp::model::urdf::loadRobotModel (device,
                    std::string (rootJointType),
                    std::string (packageName),
                    std::string (modelName),
                    std::string (urdfSuffix),
                    std::string (srdfSuffix));
            // Add device to the planner
            problemSolver_->robot (device);
            problemSolver_->robot ()->controlComputation
            (model::Device::JOINT_POSITION);
        }
        catch (const std::exception& exc)
        {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
        }
    }

    void RbprmBuilder::loadFullBodyRobot(const char* robotName,
         const char* rootJointType,
         const char* packageName,
         const char* modelName,
         const char* urdfSuffix,
         const char* srdfSuffix) throw (hpp::Error)
    {
        try
        {
            model::DevicePtr_t device = model::Device::create (robotName);
            hpp::model::urdf::loadRobotModel (device,
                    std::string (rootJointType),
                    std::string (packageName),
                    std::string (modelName),
                    std::string (urdfSuffix),
                    std::string (srdfSuffix));
            fullBody_ = rbprm::RbPrmFullBody::create(device);
            problemSolver_->pathValidationType ("Discretized",0.05); // reset to avoid conflict with rbprm path
            problemSolver_->robot (fullBody_->device_);
            problemSolver_->resetProblem();
            problemSolver_->robot ()->controlComputation
            (model::Device::JOINT_POSITION);
        }
        catch (const std::exception& exc)
        {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
        }
        fullBodyLoaded_ = true;
    }

    hpp::floatSeq* RbprmBuilder::getSampleConfig(const char* limb, unsigned short sampleId) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        const T_Limb& limbs = fullBody_->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody_->device_->name());
            throw Error (err.c_str());
        }
        const RbPrmLimbPtr_t& limbPtr = lit->second;
        hpp::floatSeq *dofArray;
        Eigen::VectorXd config = fullBody_->device_->currentConfiguration ();
        if(sampleId > limbPtr->sampleContainer_.samples_.size())
        {
            std::string err("Limb " + std::string(limb) + "does not have samples.");
            throw Error (err.c_str());
        }
        const sampling::Sample& sample = limbPtr->sampleContainer_.samples_[sampleId];
        config.segment(sample.startRank_, sample.length_) = sample.configuration_;
        dofArray = new hpp::floatSeq();
        dofArray->length(_CORBA_ULong(config.rows()));
        for(std::size_t i=0; i< _CORBA_ULong(config.rows()); i++)
          (*dofArray)[(_CORBA_ULong)i] = config [i];
        return dofArray;
    }


    hpp::floatSeq* RbprmBuilder::getSamplePosition(const char* limb, unsigned short sampleId) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        const T_Limb& limbs = fullBody_->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody_->device_->name());
            throw Error (err.c_str());
        }
        const RbPrmLimbPtr_t& limbPtr = lit->second;
        hpp::floatSeq *dofArray;
        if(sampleId > limbPtr->sampleContainer_.samples_.size())
        {
            std::string err("Limb " + std::string(limb) + "does not have samples.");
            throw Error (err.c_str());
        }
        const sampling::Sample& sample = limbPtr->sampleContainer_.samples_[sampleId];
        const fcl::Vec3f& position = sample.effectorPosition_;
        dofArray = new hpp::floatSeq();
        dofArray->length(_CORBA_ULong(3));
        for(std::size_t i=0; i< 3; i++)
          (*dofArray)[(_CORBA_ULong)i] = position [i];
        return dofArray;
    }

    model::Configuration_t dofArrayToConfig (const model::DevicePtr_t& robot,
      const hpp::floatSeq& dofArray)
    {
        std::size_t configDim = (std::size_t)dofArray.length();
        // Get robot
        if (!robot) {
            throw hpp::Error ("No robot in problem solver.");
        }
        std::size_t deviceDim = robot->configSize ();
        // Fill dof vector with dof array.
        model::Configuration_t config (configDim);// config.resize (configDim);
        for (std::size_t iDof = 0; iDof < configDim; iDof++) {
            config [iDof] = dofArray[iDof];
        }
        // fill the vector by zero
        hppDout (info, "config dimension: " <<configDim
           <<",  deviceDim "<<deviceDim);
        if(configDim != deviceDim){
            throw hpp::Error ("dofVector Does not match");
        }
        return config;
    }

    std::vector<model::Configuration_t> doubleDofArrayToConfig (const model::DevicePtr_t& robot,
      const hpp::floatSeqSeq& doubleDofArray)
    {
        std::size_t configsDim = (std::size_t)doubleDofArray.length();
        std::vector<model::Configuration_t> res;
        for (std::size_t iConfig = 0; iConfig < configsDim; iConfig++)
        {
            res.push_back(dofArrayToConfig(robot, doubleDofArray[iConfig]));
        }
        return res;
    }

    std::vector<std::string> stringConversion(const hpp::Names_t& dofArray)
    {
        std::vector<std::string> res;
        std::size_t dim = (std::size_t)dofArray.length();
        for (std::size_t iDof = 0; iDof < dim; iDof++)
        {
            res.push_back(std::string(dofArray[iDof]));
        }
        return res;
    }

    std::vector<double> doubleConversion(const hpp::floatSeq& dofArray)
    {
        std::vector<double> res;
        std::size_t dim = (std::size_t)dofArray.length();
        for (std::size_t iDof = 0; iDof < dim; iDof++)
        {
            res.push_back(dofArray[iDof]);
        }
        return res;
    }


    void RbprmBuilder::setFilter(const hpp::Names_t& roms) throw (hpp::Error)
    {
        bindShooter_.romFilter_ = stringConversion(roms);
    }


    void RbprmBuilder::setNormalFilter(const char* romName, const hpp::floatSeq& normal, double range) throw (hpp::Error)
    {
        std::string name(romName);
        bindShooter_.normalFilter_.erase(name);
        fcl::Vec3f dir;
        for(std::size_t i =0; i <3; ++i)
        {
            dir[i] = normal[i];
        }
        NormalFilter filter(dir,range);
        bindShooter_.normalFilter_.insert(std::make_pair(name,filter));
    }

    void RbprmBuilder::boundSO3(const hpp::floatSeq& limitszyx) throw (hpp::Error)
    {
        std::vector<double> limits = doubleConversion(limitszyx);
        if(limits.size() !=6)
        {
            throw Error ("Can not bound SO3, array of 6 double required");
        }
        bindShooter_.so3Bounds_ = limits;

    }

    hpp::floatSeq* RbprmBuilder::generateContacts(const hpp::floatSeq& configuration, const hpp::floatSeq& direction) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f dir;
            for(std::size_t i =0; i <3; ++i)
            {
                dir[i] = direction[i];
            }
            model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
            rbprm::State state = rbprm::ComputeContacts(fullBody_,config,
                                            problemSolver_->collisionObstacles(), dir);
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(_CORBA_ULong(state.configuration_.rows()));
            for(std::size_t i=0; i< _CORBA_ULong(config.rows()); i++)
              (*dofArray)[(_CORBA_ULong)i] = state.configuration_ [i];
            return dofArray;
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    hpp::floatSeq* RbprmBuilder::getContactSamplesIds(const char* limbname,
                                        const hpp::floatSeq& configuration,
                                        const hpp::floatSeq& direction) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f dir;
            for(std::size_t i =0; i <3; ++i)
            {
                dir[i] = direction[i];
            }
            model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
            model::Configuration_t save = fullBody_->device_->currentConfiguration();
            fullBody_->device_->currentConfiguration(config);

            sampling::T_OctreeReport finalSet;
            rbprm::T_Limb::const_iterator lit = fullBody_->GetLimbs().find(std::string(limbname));
            if(lit == fullBody_->GetLimbs().end())
            {
                throw std::runtime_error ("Impossible to find limb for joint "
                                          + std::string(limbname) + " to robot; limb not defined.");
            }
            const RbPrmLimbPtr_t& limb = lit->second;
            fcl::Transform3f transform = limb->limb_->robot()->rootJoint()->childJoint(0)->currentTransformation (); // get root transform from configuration
            std::vector<sampling::T_OctreeReport> reports(problemSolver_->collisionObstacles().size());
            std::size_t i (0);
            //#pragma omp parallel for
            for(model::ObjectVector_t::const_iterator oit = problemSolver_->collisionObstacles().begin();
                oit != problemSolver_->collisionObstacles().end(); ++oit, ++i)
            {
                sampling::GetCandidates(limb->sampleContainer_, transform, transform, *oit, dir, reports[i]);
            }
            for(std::vector<sampling::T_OctreeReport>::const_iterator cit = reports.begin();
                cit != reports.end(); ++cit)
            {
                finalSet.insert(cit->begin(), cit->end());
            }
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(finalSet.size());
            sampling::T_OctreeReport::const_iterator candCit = finalSet.begin();
            for(std::size_t i=0; i< _CORBA_ULong(finalSet.size()); ++i, ++candCit)
            {
              (*dofArray)[(_CORBA_ULong)i] = candCit->sample_->id_;
            }
            fullBody_->device_->currentConfiguration(save);
            return dofArray;
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    void RbprmBuilder::addLimb(const char* id, const char* limb, const char* effector, const hpp::floatSeq& offset, const hpp::floatSeq& normal, double x, double y,
                               unsigned short samples, const char* heuristicName, double resolution, const char *contactType) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f off, norm;
            for(std::size_t i =0; i <3; ++i)
            {
                off[i] = offset[i];
                norm[i] = normal[i];
            }
            ContactType cType = hpp::rbprm::_6_DOF;
            if(std::string(contactType) == "_3_DOF")
            {
                cType = hpp::rbprm::_3_DOF;
            }
            fullBody_->AddLimb(std::string(id), std::string(limb), std::string(effector), off, norm, x, y, problemSolver_->collisionObstacles(), samples,heuristicName,resolution,cType);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void SetPositionAndNormal(rbprm::State& state, hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs)
    {
        core::Configuration_t old = fullBody->device_->currentConfiguration();
        model::Configuration_t config = dofArrayToConfig (fullBody->device_, configuration);
        fullBody->device_->currentConfiguration(config);
        fullBody->device_->computeForwardKinematics();
        std::vector<std::string> names = stringConversion(contactLimbs);
        for(std::vector<std::string>::const_iterator cit = names.begin(); cit != names.end();++cit)
        {
            rbprm::T_Limb::const_iterator lit = fullBody->GetLimbs().find(*cit);
            if(lit == fullBody->GetLimbs().end())
            {
                throw std::runtime_error ("Impossible to find limb for joint "
                                          + (*cit) + " to robot; limb not defined");
            }
            const core::JointPtr_t joint = fullBody->device_->getJointByName(lit->second->effector_->name());
            const fcl::Transform3f& transform =  joint->currentTransformation ();
            const fcl::Matrix3f& rot = transform.getRotation();
            state.contactPositions_[*cit] = transform.getTranslation();
            state.contactRotation_[*cit] = rot;
            state.contactNormals_[*cit] = fcl::Vec3f(rot(0,2),rot(1,2), rot(2,2));
            state.contacts_[*cit] = true;
            state.contactOrder_.push(*cit);
        }        
        state.nbContacts = state.contactNormals_.size() ;
        state.configuration_ = config;
        state.robustness =  stability::IsStable(fullBody,state);
        state.stable = state.robustness >= 0;
        fullBody->device_->currentConfiguration(old);
    }

    void RbprmBuilder::setStartState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error)
    {
        try
        {
            SetPositionAndNormal(startState_,fullBody_, configuration, contactLimbs);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void RbprmBuilder::setEndState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error)
    {
        try
        {
            SetPositionAndNormal(endState_,fullBody_, configuration, contactLimbs);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    floatSeqSeq* RbprmBuilder::interpolateConfigs(const hpp::floatSeqSeq& configs, double robustnessTreshold) throw (hpp::Error)
    {
        try
        {
            if(startState_.configuration_.rows() == 0)
            {
                throw std::runtime_error ("Start state not initialized, can not interpolate ");
            }
            if(endState_.configuration_.rows() == 0)
            {
                throw std::runtime_error ("End state not initialized, can not interpolate ");
            }
            hpp::rbprm::RbPrmInterpolationPtr_t interpolator = rbprm::RbPrmInterpolation::create(fullBody_,startState_,endState_);
            std::vector<model::Configuration_t> configurations = doubleDofArrayToConfig(fullBody_->device_, configs);
            lastStatesComputed_ = interpolator->Interpolate(problemSolver_->collisionObstacles(),configurations,robustnessTreshold);
            hpp::floatSeqSeq *res;
            res = new hpp::floatSeqSeq ();

            res->length (lastStatesComputed_.size ());
            std::size_t i=0;
            std::size_t id = 0;
            for(std::vector<State>::const_iterator cit = lastStatesComputed_.begin(); cit != lastStatesComputed_.end(); ++cit, ++id)
            {
                std::cout << "ID " << id;
                cit->print();
                const core::Configuration_t config = cit->configuration_;
                _CORBA_ULong size = (_CORBA_ULong) config.size ();
                double* dofArray = hpp::floatSeq::allocbuf(size);
                hpp::floatSeq floats (size, size, dofArray, true);
                //convert the config in dofseq
                for (model::size_type j=0 ; j < config.size() ; ++j) {
                  dofArray[j] = config [j];
                }
                (*res) [i] = floats;
                ++i;
            }
            return res;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    floatSeqSeq* RbprmBuilder::interpolate(double timestep, double path, double robustnessTreshold) throw (hpp::Error)
    {
        try
        {
        int pathId = int(path);
        if(startState_.configuration_.rows() == 0)
        {
            throw std::runtime_error ("Start state not initialized, can not interpolate ");
        }
        if(endState_.configuration_.rows() == 0)
        {
            throw std::runtime_error ("End state not initialized, can not interpolate ");
        }

        if(problemSolver_->paths().size() <= pathId)
        {
            throw std::runtime_error ("No path computed, cannot interpolate ");
        }

        hpp::rbprm::RbPrmInterpolationPtr_t interpolator = rbprm::RbPrmInterpolation::create(fullBody_,startState_,endState_,problemSolver_->paths()[pathId]);
        lastStatesComputed_ = interpolator->Interpolate(problemSolver_->collisionObstacles(),timestep,robustnessTreshold);

        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();

        res->length (lastStatesComputed_.size ());
        std::size_t i=0;
        std::size_t id = 0;
        for(std::vector<State>::const_iterator cit = lastStatesComputed_.begin(); cit != lastStatesComputed_.end(); ++cit, ++id)
        {
            std::cout << "ID " << id;
            cit->print();
            const core::Configuration_t config = cit->configuration_;
            _CORBA_ULong size = (_CORBA_ULong) config.size ();
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for (model::size_type j=0 ; j < config.size() ; ++j) {
              dofArray[j] = config [j];
            }
            (*res) [i] = floats;
            ++i;
        }
        return res;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void RbprmBuilder::saveComputedStates(const char* outfilename) throw (hpp::Error)
    {
        std::stringstream ss;
        ss << lastStatesComputed_.size()-2 << "\n";
        std::vector<rbprm::State>::iterator cit = lastStatesComputed_.begin()+1;
        int i = 0;
        ss << i++ << " ";
        cit->print(ss);
        for(std::vector<rbprm::State>::iterator cit2 = lastStatesComputed_.begin()+2;
            cit2 != lastStatesComputed_.end()-1; ++cit2, ++cit, ++i)
        {
            cit2->robustness = stability::IsStable(this->fullBody_, *cit2);
            ss << i<< " ";
            cit2->print(ss,*cit);
        }
        std::ofstream outfile;
        outfile.open(outfilename);
        if (outfile.is_open())
        {
            outfile << ss.rdbuf();
            outfile.close();
        }
        else
        {
            std::string error("Can not open outfile " + std::string(outfilename));
            throw Error(error.c_str());
        }
    }

    hpp::floatSeqSeq* RbprmBuilder::GetOctreeBoxes(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error)
    {
        try
        {
        model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
        model::Configuration_t save = fullBody_->device_->currentConfiguration();
        fullBody_->device_->currentConfiguration(config);
        fullBody_->device_->computeForwardKinematics();
        const std::map<std::size_t, fcl::CollisionObject*>& boxes =
                fullBody_->GetLimbs().at(std::string(limbName))->sampleContainer_.boxes_;
        const double resolution = fullBody_->GetLimbs().at(std::string(limbName))->sampleContainer_.resolution_;
        std::size_t i =0;
        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();
        res->length (boxes.size ());
        for(std::map<std::size_t, fcl::CollisionObject*>::const_iterator cit = boxes.begin();
            cit != boxes.end();++cit,++i)
        {
            fcl::Vec3f position = (*cit->second).getTranslation();
            _CORBA_ULong size = (_CORBA_ULong) 4;
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for (model::size_type j=0 ; j < 3 ; ++j) {
                dofArray[j] = position[j];
            }
            dofArray[3] = resolution;
            (*res) [i] = floats;
        }
        fullBody_->device_->currentConfiguration(save);
        fullBody_->device_->computeForwardKinematics();
        return res;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::getOctreeTransform(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error)
    {
        try{
        model::Configuration_t config = dofArrayToConfig (fullBody_->device_, configuration);
        model::Configuration_t save = fullBody_->device_->currentConfiguration();
        fullBody_->device_->currentConfiguration(config);
        fullBody_->device_->computeForwardKinematics();
        const hpp::rbprm::RbPrmLimbPtr_t limb =fullBody_->GetLimbs().at(std::string(limbName));
        const fcl::Transform3f transform = limb->octreeRoot();
        const fcl::Quaternion3f& quat = transform.getQuatRotation();
        const fcl::Vec3f& position = transform.getTranslation();
        hpp::floatSeq *dofArray;
        dofArray = new hpp::floatSeq();
        dofArray->length(_CORBA_ULong(7));
        for(std::size_t i=0; i< 3; i++)
          (*dofArray)[(_CORBA_ULong)i] = position [i];
        for(std::size_t i=0; i< 4; i++)
          (*dofArray)[(_CORBA_ULong)i+3] = quat [i];
        fullBody_->device_->currentConfiguration(save);
        fullBody_->device_->computeForwardKinematics();
        return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void RbprmBuilder::SetProblemSolver (hpp::core::ProblemSolverPtr_t problemSolver)
    {
        problemSolver_ = problemSolver;
        bindShooter_.problemSolver_ = problemSolver;
        //bind shooter creator to hide problem as a parameter and respect signature

        // add rbprmshooter
        problemSolver->addConfigurationShooterType("RbprmShooter",
                                                   boost::bind(&BindShooter::create, boost::ref(bindShooter_), _1));
        problemSolver->addPathValidationType("RbprmPathValidation",
					     boost::bind(&BindShooter::createPathValidation, boost::ref(bindShooter_), _1, _2));
    }

      // --------------------------------------------------------------------
  
      void RbprmBuilder::setRbShooter () throw (hpp::Error)
      {
	try {
	  core::Configuration_t q_proj;
	  core::DevicePtr_t robot = problemSolver_->robot ();
	  rbShooter_ = bindShooter_.create (robot);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------
    
      hpp::floatSeq* RbprmBuilder::rbShoot () throw (hpp::Error)
      {
	try {
	  if (!rbShooter_)
	    throw std::runtime_error ("rb shooter was not set");
	  core::Configuration_t q_proj;
	  core::DevicePtr_t robot = problemSolver_->robot ();
	  q_proj = *(rbShooter_->shoot ());
	  hppDout (info, "q_proj: " << displayConfig (q_proj));

	  hpp::floatSeq *dofArray_out = 0x0;
	  dofArray_out = new hpp::floatSeq();
	  dofArray_out->length (robot->configSize ());
	  for(std::size_t i=0; i<robot->configSize (); i++)
	    (*dofArray_out)[i] = q_proj (i);
	  return dofArray_out;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      // TODO: use ROM while shifting toward obstacle
      hpp::floatSeq* RbprmBuilder::projectOnObstacle
      (const hpp::floatSeq& dofArray, const double dist) throw (hpp::Error)
      {
	core::DevicePtr_t robot = problemSolver_->robot ();
	core::Configuration_t q = dofArrayToConfig (robot, dofArray);
	core::Configuration_t q_proj;
	model::RbPrmDevicePtr_t rbRobot = boost::static_pointer_cast<hpp::model::RbPrmDevice>(robot);
	std::vector<std::string> filter = bindShooter_.romFilter_;
	const std::map<std::string, rbprm::NormalFilter>& normalFilter = bindShooter_.normalFilter_;
	rbprm::ProjectionShooterPtr_t shooter  = 
	  rbprm::ProjectionShooter::create(robot, *(problemSolver_->problem()),
					   dist, rbRobot, filter, 
					   normalFilter);
	q_proj = shooter->project (q);
	hppDout (info, "q_proj: " << displayConfig (q_proj));

	// Try to rotate the robot manually according to surface normal info
	// a priori non-needed if orientation in projection-shooter
	// orientation done before proj to correct the distance
	//q_proj = rbprm::setOrientation (robot, q_proj);

	hpp::floatSeq *dofArray_out = 0x0;
	dofArray_out = new hpp::floatSeq();
	dofArray_out->length (robot->configSize ());
	for(std::size_t i=0; i<robot->configSize (); i++)
	  (*dofArray_out)[i] = q_proj (i);
	return dofArray_out;
      }

      // --------------------------------------------------------------------
      
      hpp::floatSeq* RbprmBuilder::setOrientation(const hpp::floatSeq& dofArray)
        throw (hpp::Error)
      {
	core::DevicePtr_t robot = problemSolver_->robot ();
	core::Configuration_t q = dofArrayToConfig (problemSolver_->robot (),
						     dofArray);
	q = rbprm::setOrientation (robot, q);
	hpp::floatSeq *dofArray_out = 0x0;
	dofArray_out = new hpp::floatSeq();
	dofArray_out->length (robot->configSize ());
	for(std::size_t i=0; i<robot->configSize (); i++)
	  (*dofArray_out)[i] = q (i);
	return dofArray_out;
      }
	  
      // --------------------------------------------------------------------
	  
      void RbprmBuilder::isRbprmValid (const hpp::floatSeq& dofArray,
				       CORBA::Boolean& trunkValidity,
				       CORBA::Boolean& romValidity,
				       CORBA::String_out report)
	throw (hpp::Error) {
	try {
	  core::DevicePtr_t robot = problemSolver_->robot ();
	  model::RbPrmDevicePtr_t rbRobot = boost::static_pointer_cast<model::RbPrmDevice>(robot);
	  const std::vector<std::string>& filter = bindShooter_.romFilter_;
	  const std::map<std::string, rbprm::NormalFilter>& normalFilters = 
	    bindShooter_.normalFilter_;
	  core::ValidationReportPtr_t valReport, unusedreport, valReport2;
	  rbprm::RbPrmValidationPtr_t validator = rbprm::RbPrmValidation::create
	    (rbRobot, filter, normalFilters);
	  core::Configuration_t q = dofArrayToConfig (problemSolver_->robot (),
						      dofArray);
	  trunkValidity = validator->trunkValidation_->validate(q, valReport);
	  romValidity = validator->validateRoms(q, filter, valReport);
	  CORBA::Boolean romValidity3 = validator->validateRoms(q, filter, valReport);
	  CORBA::Boolean romValidity2 = validator->validate(q, unusedreport, filter);
	  CORBA::Boolean romValidity4 = validator->validateRoms(q, valReport2);
	  report = CORBA::string_dup ("");

	  for(int i = 0; i < filter.size (); i++) {
	    hppDout (info, "bindShooter_.romFilter_= " << bindShooter_.romFilter_ [i]);
	    hppDout (info, "filter= " << filter [i]);
	  }

	  if(!trunkValidity || !romValidity) {
	    hppDout (info, "config is not RB-valid");
	    hppDout (info, "trunkValidity= " << trunkValidity);
	    hppDout (info, "romValidity= " << romValidity);
	    hppDout (info, "romValidity2= " << romValidity2);
	    hppDout (info, "romValidity3= " << romValidity3);
	    hppDout (info, "romValidity4= " << romValidity4);
	    std::ostringstream oss;
	    oss << *valReport;
	    report = CORBA::string_dup(oss.str ().c_str ());
	    oss << *valReport2;
	    report = CORBA::string_dup(oss.str ().c_str ());
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // --------------------------------------------------------------------

      void RbprmBuilder::interpolateBallisticPath (CORBA::UShort pathId)
	throw (hpp::Error){
        try {
	  std::size_t pid = (std::size_t) pathId;
	  if(startState_.configuration_.rows() == 0)
	    {
	      throw std::runtime_error ("Start state not initialized, can not interpolate ");
	    }
	  if(endState_.configuration_.rows() == 0)
	    {
	      throw std::runtime_error ("End state not initialized, can not interpolate ");
	    }

	  if(problemSolver_->paths().size() <= pid)
	    {
	      throw std::runtime_error ("No path computed, cannot interpolate ");
	    }

	  hpp::rbprm::BallisticInterpolationPtr_t interpolator = 
	    rbprm::BallisticInterpolation::create(fullBody_, startState_,
						  endState_,
						  problemSolver_->paths()[pid]);
	  core::PathVectorPtr_t newPath = interpolator->InterpolateFullPath(problemSolver_->collisionObstacles());
	  std::size_t newPathId = problemSolver_->addPath (newPath);
	  hppDout (info, "newPath Id is: " << newPathId);
	}
	catch(std::runtime_error& e)
	  {
	    throw Error(e.what());
	  }
      }

      // --------------------------------------------------------------------

      hpp::floatSeqSeq* RbprmBuilder::generateWaypointContacts
      (CORBA::UShort pathId) throw (hpp::Error){
        try {
	  std::size_t pid = (std::size_t) pathId;
	  if(problemSolver_->paths().size() <= pid) {
	    throw std::runtime_error ("No path computed");
	  }

	  core::DevicePtr_t robot = problemSolver_->robot ();
	  const CORBA::ULong configSize = robot->configSize(); // with ECS
	  const CORBA::ULong ecsSize = robot->extraConfigSpace ().dimension ();
	  core::PathVectorPtr_t path = problemSolver_->paths () [pid];
	  std::size_t num_subpaths  = (*path).numberPaths ();
	  std::size_t num_waypoints  = num_subpaths - 1;
	  bool success;
	  hpp::floatSeqSeq *configSequence;
	  configSequence = new hpp::floatSeqSeq ();
	  configSequence->length ((CORBA::ULong) num_waypoints);

	  for (std::size_t i = 1; i < num_subpaths; i++) {
	    core::PathPtr_t subpath = (*path).pathAtRank (i); // trunk-size!!
	    const std::size_t trunkSize = subpath->initial ().size ();
	    hppDout (info, "trunkSize: " << trunkSize);
	    model::Configuration_t q1 (configSize);
	    hppDout (info, "waypoint before fill: " << displayConfig(subpath->initial ()));
	    for (std::size_t j = 0; j < configSize - ecsSize; j++) {
	      if (j < trunkSize)
		q1 [j] = subpath->initial () [j];
	      else
		q1 [j] = 0;
	    }
	    for (std::size_t k = 0; k < ecsSize; k++)
	      q1 [configSize - ecsSize + k] = subpath->initial () [trunkSize - ecsSize + k];
	    hppDout (info, "waypoint before contacts: " << displayConfig(q1));
	    fcl::Vec3f dir;
	    dir [0] = 0; dir [1] = 0; dir [2] = 1;
	    rbprm::State state =
	      rbprm::ComputeContacts(fullBody_, q1,
				     problemSolver_->collisionObstacles(), dir);
	    model::Configuration_t q = state.configuration_;
	    hppDout (info, "waypoint after contacts: " << displayConfig(q));
	    hpp::floatSeq* dofArray;
	    dofArray = new hpp::floatSeq ();
	    dofArray->length (configSize);
	    for (std::size_t j = 0; j < robot->configSize(); j++) {
	      dofArray [j] = q (j);
	      hppDout (info, "q [j]: " << q (j));
	    }
	    if (i - 1 < num_waypoints)
	      (*configSequence) [(CORBA::ULong) i - 1] = *dofArray;
	    hppDout (info, "i: " << (CORBA::ULong) i);
	    //hppDout (info, "*dofArray: " << *dofArray);
	  }
	  return configSequence;
	}
	catch(std::runtime_error& e)
	  {
	    throw Error(e.what());
	  }
      }

      // --------------------------------------------------------------------

      hpp::floatSeq* RbprmBuilder::fillConfiguration
      (const hpp::floatSeq& dofArray, const CORBA::UShort fullSize)
	throw (hpp::Error)
      {
	core::DevicePtr_t robot = problemSolver_->robot ();
	core::Configuration_t q = dofArrayToConfig (robot, dofArray);
	std::size_t trunkSize = q.size ();
	std::size_t ecsSize = robot->extraConfigSpace ().dimension ();
	core::Configuration_t result (fullSize);
	hppDout (info, "original config= " << displayConfig (q));
	for (std::size_t j = 0; j < fullSize - ecsSize; j++) {
	  if (j < trunkSize)
	    result [j] = q [j];
	  else
	    result [j] = 0;
	}
	// copy extra-configs at the end of the config
	for (std::size_t k = 0; k < ecsSize; k++)
	  result [fullSize - ecsSize + k] = q [trunkSize - ecsSize + k];
	hppDout (info, "filled config= " << displayConfig (result));
	
	hpp::floatSeq *dofArray_out = 0x0;
	dofArray_out = new hpp::floatSeq();
	dofArray_out->length (robot->configSize ());
	for(std::size_t i=0; i<robot->configSize (); i++)
	  (*dofArray_out)[i] = result (i);
	return dofArray_out;
      }

    } // namespace impl
  } // namespace rbprm
} // namespace hpp
