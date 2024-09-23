#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
    namespace geometric
    {
        // Random Tree Planner (RTP)
        class RTP : public base::Planner
        {
        public:
            // Constructor: Initialize RTP with space information and option to add intermediate states
            RTP(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);

            // Destructor
            ~RTP() override;

            // Get planner data for visualization or debugging
            void getPlannerData(base::PlannerData &data) const override;

            // Solve method implementing the core RTP planning algorithm
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            // Clear memory and reset planner
            void clear() override;

            // Set goal bias (probability of selecting the goal during random sampling)
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            // Get goal bias
            double getGoalBias() const
            {
                return goalBias_;
            }

            // Get whether intermediate states are added during planning
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            // Set whether to add intermediate states during planning
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            // Set maximum distance between states during planning
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            // Get maximum distance between states during planning
            double getRange() const
            {
                return maxDistance_;
            }

            // Template to set nearest neighbor structure
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            // Setup planner (initialize necessary data structures and parameters)
            void setup() override;

        protected:
            // Motion class to represent a node in the tree
            class Motion
            {
            public:
                Motion() = default;

                // Constructor: Initialize motion with allocated state
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                // Destructor: Free motion state
                ~Motion() = default;

                // State associated with this motion (node)
                base::State *state{nullptr};

                // Parent motion (used to trace the path back to the start)
                Motion *parent{nullptr};
            };

            // Free memory allocated for motions
            void freeMemory();

            // Distance function for nearest neighbor calculations
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            // State sampler for generating random states
            base::StateSamplerPtr sampler_;

            // Nearest neighbor data structure to store motions (tree nodes)
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            // Goal bias probability (chance to directly sample the goal)
            double goalBias_{.05};

            // Maximum allowable distance between states during tree expansion
            double maxDistance_{0.};

            // Flag to indicate whether intermediate states should be added during planning
            bool addIntermediateStates_;

            // Random number generator
            RNG rng_;

            // Last goal motion (used for constructing the final path)
            Motion *lastGoalMotion_{nullptr};
        };

    } // namespace geometric
} // namespace ompl

#endif // RANDOM_TREE_H