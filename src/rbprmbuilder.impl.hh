// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux
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

#ifndef HPP_RBPRM_CORBA_BUILDER_IMPL_HH
# define HPP_RBPRM_CORBA_BUILDER_IMPL_HH

# include <hpp/core/problem-solver.hh>
# include <hpp/core/path.hh>
# include "hpp/corbaserver/rbprm/rbprmbuilder.hh" //# include "rbprmbuilder.hh"
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/rbprm-shooter.hh>
# include <hpp/rbprm/rbprm-validation.hh>
# include <hpp/rbprm/sampling/analysis.hh>
# include <hpp/core/collision-path-validation-report.hh>
# include <hpp/core/problem-solver.hh>
# include <hpp/core/discretized-collision-checking.hh>
# include <hpp/core/straight-path.hh>
# include <hpp/rbprm/rbprm-path-validation.hh>
# include <hpp/fcl/BVH/BVH_model.h>
# include <hpp/core/config-validations.hh>
# include <hpp/model/configuration.hh>
# include <hpp/core/distance.hh>
# include "hpp/rbprm/fullbodyBallistic/convex-cone-intersection.hh"
# include "hpp/rbprm/interpolation/limb-rrt.hh"
# include "hpp/rbprm/planner/parabola-path.hh"

namespace hpp {
  namespace rbprm {
    namespace impl {
      using CORBA::Short;
      typedef std::map<std::string, std::vector<boost::shared_ptr<model::CollisionObject> > > affMap_t;

    struct BindAnalysis
    {
      BindAnalysis():
        refConfig_(),conf_()
      {
      }
      BindAnalysis(hpp::core::ProblemSolverPtr_t problemSolver):
        refConfig_(),conf_(),problemSolver_(problemSolver)
      {
      }
      
      double ReferenceAnalysis(const sampling::SampleDB& sampleDB, const sampling::Sample& sample)
      {
          // TODO compute distance between refConfig and sample sample.configuration_
        conf_ = sample.configuration_;
        hppDout(info,"reference config = "<<model::displayConfig(refConfig_));
        hppDout(info,"current config   = "<<model::displayConfig(conf_));
        // doesn't work anymore since we only use limb config and not full body
        //core::value_type distance = (*(problemSolver_->problem()->distance())) (conf_,refConfig_);

        assert(refConfig_.size() == conf_.size());
        assert(weight_.size() == conf_.size());
        
        //compute distance TODO : improve it
        assert(conf_.size() == refConfig_.size());
        double distance =0 ;
        double d =0;
        for(size_t i = 0 ; i<conf_.size(); i++){
            d=conf_[i] - refConfig_[i];
            distance += weight_[i] * d*d;
        }


        //return distance*1000. + sample.staticValue_;
        hppDout(info,"heuristic value = "<<-distance);
        return -distance;
      }
      
      void setConfig(model::Configuration_t ref){
        refConfig_ = ref;
      }

      void setWeight(model::Configuration_t w){
          weight_ = w;
      }
      
      core::Configuration_t refConfig_;
      core::Configuration_t conf_;
      core::Configuration_t weight_;
      hpp::core::ProblemSolverPtr_t problemSolver_;
      
    };
      
    struct BindShooter
    {
        BindShooter(const std::size_t shootLimit = 10000,
                    const std::size_t displacementLimit = 100)
            : shootLimit_(shootLimit)
            , displacementLimit_(displacementLimit), nbFilterMatch_ (0),
	      fullOrientationMode_ (false)
      {}

        hpp::rbprm::RbPrmShooterPtr_t create (const hpp::model::DevicePtr_t& robot)
        {
	  hppDout (info, "create bindshooter");
	  std::size_t effectiveNbFilterMatch (nbFilterMatch_);
	  if (nbFilterMatch_ == 0)
	    effectiveNbFilterMatch = romFilter_.size ();
	  hppDout (info, "effectiveNbFilterMatch= " << effectiveNbFilterMatch);
	  hpp::model::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::model::RbPrmDevice>(robot);
	  if (affMap_.empty ()) {
	    hppDout (info, "No affordances found. Unable to create shooter object.");
	    throw hpp::Error ("No affordances found. Unable to create shooter object.");
	  }
	  rbprm::RbPrmShooterPtr_t shooter = hpp::rbprm::RbPrmShooter::create
	    (robotcast, problemSolver_->problem ()->collisionObstacles(), affMap_, romFilter_,affFilter_,shootLimit_,displacementLimit_,effectiveNbFilterMatch);

	  shooter->fullOrientationMode (fullOrientationMode_);
	  shooter->interiorPoint (interiorPoint_);
	  hppDout (info, "fullOrientationMode = " << fullOrientationMode_);
	  hppDout (info, "interiorPoint= " << interiorPoint_);
            if(!so3Bounds_.empty())
                shooter->BoundSO3(so3Bounds_);
            return shooter;
        }

      hpp::core::PathValidationPtr_t createPathValidation (const hpp::model::DevicePtr_t& robot, const hpp::model::value_type& val)
      {
	std::size_t effectiveNbFilterMatch (nbFilterMatch_);
	if (nbFilterMatch_ == 0)
	  effectiveNbFilterMatch = romFilter_.size ();
	hppDout (info, "effectiveNbFilterMatch= " << effectiveNbFilterMatch);
            hpp::model::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::model::RbPrmDevice>(robot);
            affMap_ = problemSolver_->map
              <std::vector<boost::shared_ptr<model::CollisionObject> > > ();
            if (affMap_.empty ()) {
	      hppDout (info, "No affordances found. Unable to create Path Validaton object.");
              throw hpp::Error ("No affordances found. Unable to create Path Validaton object.");
            }
            hpp::rbprm::RbPrmValidationPtr_t validation
              (hpp::rbprm::RbPrmValidation::create(robotcast, romFilter_, affFilter_, affMap_, core::ObjectVector_t(), effectiveNbFilterMatch));
            hpp::rbprm::RbPrmPathValidationPtr_t collisionChecking = hpp::rbprm::RbPrmPathValidation::create(robot,val);
            collisionChecking->add (validation);
            problemSolver_->problem()->configValidation(core::ConfigValidations::create ());
            problemSolver_->problem()->configValidations()->add(validation);
            return collisionChecking;
        }

        hpp::core::ProblemSolverPtr_t problemSolver_;
        std::vector<std::string> romFilter_;
        std::map<std::string, std::vector<std::string> > affFilter_;
        std::size_t shootLimit_;
        std::size_t displacementLimit_;
        std::vector<double> so3Bounds_;
        std::size_t nbFilterMatch_;
        bool fullOrientationMode_;
	affMap_t affMap_;
        fcl::Vec3f interiorPoint_;
    };

      // -----------------------------------------------------------------
      class RbprmBuilder : public virtual POA_hpp::corbaserver::rbprm::RbprmBuilder
      {
        public:
        RbprmBuilder ();

        virtual void loadRobotRomModel (const char* robotName,
                 const char* rootJointType,
                 const char* packageName,
                 const char* modelName,
                 const char* urdfSuffix,
                 const char* srdfSuffix) throw (hpp::Error);

        virtual void loadRobotCompleteModel (const char* robotName,
                 const char* rootJointType,
                 const char* packageName,
                 const char* modelName,
                 const char* urdfSuffix,
                 const char* srdfSuffix) throw (hpp::Error);


        virtual void loadFullBodyRobot (const char* robotName,
                 const char* rootJointType,
                 const char* packageName,
                 const char* modelName,
                 const char* urdfSuffix,
                 const char* srdfSuffix) throw (hpp::Error);

        virtual void loadFullBodyRobotFromExistingRobot () throw (hpp::Error);


        virtual void setFilter(const hpp::Names_t& roms) throw (hpp::Error);
				virtual void setAffordanceFilter(const char* romName, const hpp::Names_t& affordances) throw (hpp::Error);
        virtual void boundSO3(const hpp::floatSeq& limitszyx) throw (hpp::Error);


        virtual hpp::floatSeq* getSampleConfig(const char* limb, unsigned short sampleId) throw (hpp::Error);
        virtual hpp::floatSeq* getSamplePosition(const char* limb, unsigned short sampleId) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getEffectorPosition(const char* limb, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual CORBA::UShort getNumSamples(const char* limb) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeNodeIds(const char* limb) throw (hpp::Error);
        virtual double getSampleValue(const char* limb, const char* valueName, unsigned short sampleId) throw (hpp::Error);
        virtual double getEffectorDistance(unsigned short  state1, unsigned short  state2) throw (hpp::Error);

        virtual hpp::floatSeq* generateContacts(const hpp::floatSeq& configuration,
                                                const hpp::floatSeq& direction,
						const bool noStability = false,
						const unsigned short useFlexionPose = 1)
	  throw (hpp::Error);

        virtual hpp::floatSeq* generateGroundContact(const hpp::Names_t& contactLimbs) throw (hpp::Error);

        virtual hpp::floatSeq* getContactSamplesIds(const char* limb,
                                                   const hpp::floatSeq& configuration,
                                                   const hpp::floatSeq& direction) throw (hpp::Error);

        virtual hpp::floatSeq* getSamplesIdsInOctreeNode(const char* limb,
                                                   double octreeNodeId) throw (hpp::Error);

        virtual void addLimb(const char* id, const char* limb, const char* effector, const hpp::floatSeq& offset, const hpp::floatSeq& normal, double x, double y,
                             unsigned short samples, const char *heuristicName, double resolution, const char *contactType,
                             double disableEffectorCollision) throw (hpp::Error);
        virtual void addLimbDatabase(const char* databasePath, const char* id, const char* heuristicName, double loadValues, double disableEffectorCollision) throw (hpp::Error);

        virtual void setStartState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual void setEndState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual hpp::floatSeq*  computeContactForConfig(const hpp::floatSeq& configuration, const char* limbNam) throw (hpp::Error);
        virtual hpp::floatSeqSeq* computeContactPoints(unsigned short cId) throw (hpp::Error);
        virtual hpp::floatSeqSeq* computeContactPointsForLimb(unsigned short cId, const char* limbName) throw (hpp::Error);
        virtual hpp::floatSeqSeq* interpolate(double timestep, double path, double robustnessTreshold, unsigned short filterStates) throw (hpp::Error);
        virtual hpp::floatSeqSeq* interpolateConfigs(const hpp::floatSeqSeq& configs, double robustnessTreshold, unsigned short filterStates) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getContactCone(unsigned short stateId, double friction) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getContactIntermediateCone(unsigned short stateId, double friction) throw (hpp::Error);
        virtual CORBA::Short generateComTraj(const hpp::floatSeqSeq& positions, const hpp::floatSeqSeq& velocities,
                                             const hpp::floatSeqSeq& accelerations, const double dt) throw (hpp::Error);
        virtual CORBA::Short generateRootPath(const hpp::floatSeqSeq& rootPositions,
                                      const hpp::floatSeq& q1, const hpp::floatSeq& q2) throw (hpp::Error);
        virtual CORBA::Short limbRRT(double state1, double state2, unsigned short numOptimizations) throw (hpp::Error);
        virtual CORBA::Short limbRRTFromRootPath(double state1, double state2, unsigned short path, unsigned short numOptimizations) throw (hpp::Error);
        virtual CORBA::Short configToPath(const hpp::floatSeqSeq& configs) throw (hpp::Error);
        virtual CORBA::Short comRRT(double state1, double state2, unsigned short path, unsigned short numOptimizations) throw (hpp::Error);

        typedef core::PathPtr_t (*t_rrt)
            (RbPrmFullBodyPtr_t, core::ProblemPtr_t, const core::PathPtr_t,
             const  State &, const State &, const  std::size_t, const bool);

        hpp::floatSeq* rrt(t_rrt functor ,double state1,
                           unsigned short comTraj1, unsigned short comTraj2, unsigned short comTraj3,
                           unsigned short numOptimizations) throw (hpp::Error);

        virtual hpp::floatSeq* comRRTFromPos(double state1,
                                           unsigned short comTraj1,
                                           unsigned short comTraj2,
                                           unsigned short comTraj3,
                                           unsigned short numOptimizations) throw (hpp::Error);
        virtual hpp::floatSeq* effectorRRT(double state1,
                                           unsigned short comTraj1,
                                           unsigned short comTraj2,
                                           unsigned short comTraj3,
                                           unsigned short numOptimizations) throw (hpp::Error);
        virtual hpp::floatSeq* effectorRRTFromPath(double state1,
                                           unsigned short path,
                                           double path_from,
                                           double path_to,
                                           unsigned short comTraj1,
                                           unsigned short comTraj2,
                                           unsigned short comTraj3,
                                           unsigned short numOptimizations,
                                           const hpp::Names_t& trackedEffectors) throw (hpp::Error);
        virtual hpp::floatSeq* projectToCom(double state, const hpp::floatSeq& targetCom) throw (hpp::Error);
        virtual CORBA::Short createState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs, const bool isInContact = true, const double time = -1) throw (hpp::Error);
        virtual hpp::floatSeq* getConfigAtState(unsigned short stateId) throw (hpp::Error);
        double projectStateToCOMEigen(unsigned short stateId, const model::Configuration_t& com_target)throw (hpp::Error);
        virtual double projectStateToCOM(unsigned short stateId, const hpp::floatSeq& com) throw (hpp::Error);
        virtual void saveComputedStates(const char* filepath) throw (hpp::Error);
        virtual void saveLimbDatabase(const char* limbname,const char* filepath) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeBox(const char* limbName, double sampleId) throw (hpp::Error);
        virtual CORBA::Short  isLimbInContact(const char* limbName, double state) throw (hpp::Error);
        virtual CORBA::Short  isLimbInContactIntermediary(const char* limbName, double state) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getOctreeBoxes(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeTransform(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual CORBA::Short isConfigBalanced(const hpp::floatSeq& config, const hpp::Names_t& contactLimbs, double robustnessTreshold) throw (hpp::Error);
        virtual void runSampleAnalysis(const char* analysis, double isstatic) throw (hpp::Error);
        virtual hpp::floatSeq* runLimbSampleAnalysis(const char* limbname, const char* analysis, double isstatic) throw (hpp::Error);
        virtual void dumpProfile(const char* logFile) throw (hpp::Error);

      public:
        void SetProblemSolver (hpp::core::ProblemSolverPtr_t problemSolver);

	virtual hpp::floatSeq* rbShoot () throw (hpp::Error);

	hpp::floatSeq* projectOnObstacle (const hpp::floatSeq& dofArray,
					  const double dist) throw (hpp::Error)
	{
	  throw std::runtime_error("projection on obstacles NOT implemented in this version (with affordances)");
	}

	virtual hpp::floatSeq* setOrientation (const hpp::floatSeq& dofArray)
	  throw (hpp::Error);
	virtual hpp::floatSeq* computeOrientationQuat
	(const hpp::floatSeq& normal, const double theta,
	 const char* robotName) throw (hpp::Error);
	void setRbShooter () throw (hpp::Error);
	void isRbprmValid (const hpp::floatSeq& dofArray,
			   CORBA::Boolean& trunkValidity,
			   CORBA::Boolean& romValidity,
			   CORBA::String_out report) throw (hpp::Error);
	void interpolateBallisticPath (const CORBA::UShort pathId,
				       const double u_offset,
				       const CORBA::UShort maxIter,
				       const CORBA::Boolean timed = false,
				       const CORBA::Boolean comProj = false)
	  throw (hpp::Error);
	hpp::floatSeqSeq* generateWaypointContacts (CORBA::UShort pathId)
	  throw (hpp::Error);
	virtual hpp::floatSeq* fillConfiguration (const hpp::floatSeq& dofArray,
						  const CORBA::UShort fullSize)
	  throw (hpp::Error);
	void setNumberFilterMatch (const CORBA::UShort nbFilterMatch)
	  throw (hpp::Error);
	hpp::floatSeqSeq* getsubPathsV0Vimp (const char* Vquery,
					     CORBA::UShort pathId)
	  throw (hpp::Error);

	void rotateAlongPath (const CORBA::UShort pathId,
			      const bool rotateAfterJump,
			      const bool trunkOrientation,
			      const bool getCloseToContact) throw (hpp::Error);

	void setFullOrientationMode (const bool fullOrientationMode);

	void setContactSize (const double xValue, const double yValue);

	void setPose (const hpp::floatSeq& dofArray,
		      const char* poseQuery) throw (hpp::Error);

	void setMaxTakeoffVelocity (const double vmax) throw (hpp::Error);
	void setMaxLandingVelocity (const double vmax) throw (hpp::Error);
	void setFrictionCoef (const double mu) throw (hpp::Error);
	hpp::intSeq* getResultValues () throw (hpp::Error);
    //void setReferenceConfig (const hpp::floatSeq& dofArray)throw (hpp::Error);
    void addRefConfigAnalysis (const hpp::floatSeq& dofArray, const char* name)throw (hpp::Error);
    void addRefConfigAnalysisWeight (const hpp::floatSeq& dofArray, const char* name,const hpp::floatSeq& weightArray)throw (hpp::Error);

	void setFullbodyFrictionCoef (const double mu);

	void setFullbodyV0fThetaCoefs (const char* Vquery,
				       const CORBA::Boolean clean,
				       const hpp::floatSeq& Varray,
				       const double theta) throw (hpp::Error);

	hpp::floatSeq* convexConePlaneIntersection
	(const unsigned short Ncones, const hpp::floatSeqSeq& cones,
	 const double theta, const double mu) throw (hpp::Error);

	virtual hpp::floatSeqSeq* getContactCones
	(const hpp::floatSeq& dofArray, hpp::floatSeqSeq_out conePositionSeq)
	  throw (hpp::Error);
	hpp::floatSeqSeq* getlastStatesComputedTime ();
	void setFillGenerateContactState (const CORBA::Boolean b);
	void setInteriorPoint (const hpp::floatSeq& point);
	hpp::floatSeq* quaternionDifference (const hpp::floatSeq& quat1,
					     const hpp::floatSeq& quat2);
	CORBA::UShort getcentroidalConeFails ();
	CORBA::UShort getPathPlannerFails ();
	void setcentroidalConeFails (CORBA::UShort number);
	void setPathPlannerFails (CORBA::UShort number);
	CORBA::Boolean isTrunkCollisionFree (const hpp::floatSeq& config);
	void setFullbodyInteriorPoint (const hpp::floatSeq& point);
	void projectLastStatesComputedToCOM () throw (hpp::Error);
	void interpolatePathFromLastStatesComputed () throw (hpp::Error);

      private:
        /// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
        core::ProblemSolverPtr_t problemSolver_;
        model::T_Rom romDevices_;
        rbprm::RbPrmFullBodyPtr_t fullBody_;
        bool romLoaded_;
        bool fullBodyLoaded_;
        BindShooter bindShooter_;
        std::map<std::string,BindAnalysis> bindAnalysis_;
	rbprm::State generateContactState_;
        rbprm::State startState_;
        rbprm::State endState_;
        std::vector<rbprm::State> lastStatesComputed_;
        rbprm::T_StateFrame lastStateFramesComputed_; // limbRRT + ballistic
	rbprm::RbPrmShooterPtr_t rbShooter_;
        rbprm::T_StateFrame lastStatesComputedTime_; // new Pierre
        sampling::AnalysisFactory* analysisFactory_;
	core::Configuration_t extendingPose_; // parabola apex
	core::Configuration_t flexionPose_; // parabola extremity
	core::Configuration_t takeoffContactPose_; // when releasing contact
	core::Configuration_t landingContactPose_; // when starting contact
	core::Configuration_t flexionFinalPose_; // last flexion pose
	CORBA::Boolean fillGenerateContactState_;
	ParabolaPathPtr_t parabolaPath_; // parabola path that goes with StateFrames, WARNING works for 1 parabola only
      }; // class RobotBuilder
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_RBPRM_CORBA_BUILDER_IMPL_HH
