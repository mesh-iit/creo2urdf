#include <creo2urdf/ComponentIdentity.h>
#include <iDynTree/Model.h>
#include <iDynTree/FixedJoint.h>
#include <iostream>

int main() {
    try {
        const std::map<ComponentId, std::string> models{{{40}, "LINK"}, {{75}, "LINK"}};
        const auto names = resolveComponentNames(models, {{{40}, "root_link"}, {{75}, "moving_link"}}, {});
        iDynTree::Model model;
        iDynTree::Link link;
        for (const auto& occurrence : names) {
            // addLink returns an index: zero is a successful first insertion.
            const auto index = model.addLink(occurrence.second, link);
            if (index == iDynTree::LINK_INVALID_INDEX)
                throw std::runtime_error("Could not add occurrence " + occurrence.second);
        }
        if (model.getLinkIndex("root_link") != 0 || model.getNrOfLinks() != 2)
            throw std::runtime_error("Repeated-part links were not added correctly");

        iDynTree::FixedJoint joint(iDynTree::Transform::Identity());
        if (model.addJoint("root_link", "moving_link", "joint", &joint) == iDynTree::JOINT_INVALID_INDEX)
            throw std::runtime_error("Could not connect the two occurrences");
        if (model.getNrOfJoints() != 1)
            throw std::runtime_error("Missing joint");

        if (model.addLink("root_link", link) != iDynTree::LINK_INVALID_INDEX || model.getNrOfLinks() != 2)
            throw std::runtime_error("Duplicate link did not return the failure sentinel");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
