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

        // Get the dimension of this action's manifold.
        virtual int getDimension() const = 0;

        // Project the state into the lower dimensional manifold spanned by this primitive action.
        virtual void project(ompl::base::State* x_i,
                             ompl::base::State* x_o) const = 0;

        // Returns true if this action can propogate from x_i to x_f.
        virtual bool isUseful(ompl::base::State* x_i,
                              ompl::base::State* x_f) const = 0;

        // Propogate the primitive action from x_i to x_f if possible.
        virtual void propogate(ompl::base::State* x_i,
                               ompl::base::State* x_f,
                               ompl::base::State* x_o) const = 0;

    protected:
        ActionBasedStateSpacePtr _space;
    };

    // An action for moving the robot end-effector in a straight line.
    class StraightLineAction : public ActionType
    {
    public:

        StraightLineAction(ActionBasedStateSpacePtr& space,
                           const robot_model::RobotModelConstPtr& robot)
            : ActionType(space),
              _robot(robot)
        {
        }

        ~StraightLineAction() {}

        // Get the dimension of this action's manifold.
        virtual int getDimension() const;

        // Project the state into the lower dimensional manifold spanned by this primitive action.
        virtual void project(ompl::base::State* x_i,
                             ompl::base::State* x_o) const;

        // Returns true if this action can propogate from x_i to x_f.
        virtual bool isUseful(ompl::base::State* x_i,
                              ompl::base::State* x_f) const;

        // Propogate the primitive action from x_i to x_f if possible.
        virtual void propogate(ompl::base::State* x_i,
                               ompl::base::State* x_f,
                               ompl::base::State* x_o) const;

    protected:
        const robot_model::RobotModelConstPtr& _robot;
    };

    // An action type created from a primitive plan message.
    class MsgAction : public ActionType
    {
    public:
        MsgAction(ActionBasedStateSpacePtr& space,
                  const apc_msgs::PrimitivePlan& action);
        ~MsgAction() {}

        // Setup this primitive action.
        void setMsg(const apc_msgs::PrimitivePlan& action);

        // Get the dimension of this action's manifold.
        virtual int getDimension() const;

        // Project the state into the lower dimensional manifold spanned by this primitive action.
        virtual void project(ompl::base::State* x_i,
                             ompl::base::State* x_o) const;

        // Returns true if this action can propogate from x_i to x_f.
        virtual bool isUseful(ompl::base::State* x_i,
                              ompl::base::State* x_f) const;

        // Propogate the primitive action from x_i to x_f if possible.
        virtual void propogate(ompl::base::State* x_i,
                               ompl::base::State* x_f,
                               ompl::base::State* x_o) const;

    protected:
        apc_msgs::PrimitivePlan _action;
    };
}
