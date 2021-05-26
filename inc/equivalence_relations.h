#ifndef TEB_EQUIVALENCE_RELATIONS_H
#define TEB_EQUIVALENCE_RELATIONS_H

#include <boost/shared_ptr.hpp>

namespace teb_local_planner
{

/**
 * @class EquivalenceClass
 * @brief Abstract class that defines an interface for computing and comparing equivalence classes
 *
 * Equivalence relations are utilized in order to test if two trajectories are belonging to the same
 * equivalence class w.r.t. the current obstacle configurations. A common equivalence relation is
 * the concept of homotopy classes. All trajectories belonging to the same homotopy class
 * can CONTINUOUSLY be deformed into each other without intersecting any obstacle. Hence they likely
 * share the same local minimum after invoking (local) trajectory optimization. A weaker equivalence relation
 * is defined by the concept of homology classes (e.g. refer to HSignature).
 *
 * Each EquivalenceClass object (or subclass) stores a candidate value which might be compared to another EquivalenceClass object.
 *
 * @remarks Currently, the computeEquivalenceClass method is not available in the generic interface EquivalenceClass.
 *          Call the "compute"-methods directly on the subclass.
 */
    class EquivalenceClass
    {
    public:

        /**
         * @brief Default constructor
         */
        EquivalenceClass()
        {}

        /**
         * @brief virtual destructor
         */
        virtual ~EquivalenceClass()
        {}

        /**
         * @brief Check if two candidate classes are equivalent
         * @param other The other equivalence class to test with
         */
        virtual bool isEqual(const EquivalenceClass &other) const = 0;

        /**
         * @brief Check if the equivalence value is detected correctly
         * @return Returns false, if the equivalence class detection failed, e.g. if nan- or inf values occur.
         */
        virtual bool isValid() const = 0;

        /**
         * @brief Check if the trajectory is non-looping around an obstacle
         * @return Returns false, if the trajectory loops around an obstacle
         */
        virtual bool isReasonable() const = 0;

    };

    using EquivalenceClassPtr = boost::shared_ptr<EquivalenceClass>;
}
#endif //TEB_EQUIVALENCE_RELATIONS_H
