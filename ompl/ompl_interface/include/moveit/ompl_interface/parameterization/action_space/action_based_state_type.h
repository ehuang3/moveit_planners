/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_ACTION_SPACE_ACTION_BASED_STATE_TYPE_
#define MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_ACTION_SPACE_ACTION_BASED_STATE_TYPE_

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <apc_msgs/PrimitivePlan.h>

namespace ompl_interface
{
    class ActionType
    {
    public:
        virtual ActionType(ActionBasedStateSpacePtr& space);
        virtual ~ActionType();

        // Project the state into the lower dimensional manifold spanned by this primitive action.
        virtual void project(ompl::base::State* x) const = 0;

        // Returns true if this action can propogate from x_i to x_f.
        virtual bool isUseful(ompl::base::State* x_i,
                              ompl::base::State* x_f) const = 0;

        // Propogate the primitive action from x_i to x_f if possible.
        virtual void propogate(ompl::base::State* x_i,
                               ompl::base::State* x_f,
                               ompl::base::State* x_o) const = 0;

    private:
        ActionBasedStateSpacePtr _space;
    };

    /**< \brief boost::shared_ptr to an action-based state. */
    typedef boost::shared_ptr<ActionType> ActionTypePtr;

    /**< \brief boost::shared_ptr to a constant action-based state. */
    typedef boost::shared_ptr<const ActionType> ActionTypeConstPtr;

    struct ActionBasedStateType : public ModelBasedStateSpace::StateType
    {
        // Constructor.
        ActionBasedStateType();
        virtual ~ActionBasedStateType();



        // virtual void serialize(void* serialization);
        // virtual void deserialize(void *serialization);
        // virtual int getDimensions();

        // Name of the primitive action associated with this state.
        std::string action_name;

        // Name of the attached object.
        std::string object_name;
    };

    /**< \brief boost::shared_ptr to an action-based state. */
    typedef boost::shared_ptr<ActionBasedStateType> ActionBasedStateTypePtr;

    /**< \brief boost::shared_ptr to a constant action-based state. */
    typedef boost::shared_ptr<const ActionBasedStateType> ActionBasedStateTypeConstPtr;

}

#endif
