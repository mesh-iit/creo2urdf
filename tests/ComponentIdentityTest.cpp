#include <creo2urdf/ComponentIdentity.h>
#include <iostream>

static void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template<class Function> void rejects(Function function, const char* message) {
    bool rejected = false;
    try { function(); } catch (const std::runtime_error&) { rejected = true; }
    check(rejected, message);
}

int main() {
    try {
        struct ModelWrapper {
            int type;
            std::string name;
            int GetType() const { return type; }
            const char* GetFullName() const { return name.c_str(); }
        };
        ModelWrapper retrieved{1, "LINK"}, selected{1, "LINK"}, assembly{2, "LINK"}, other{1, "OTHER"};
        check(sameComponentModel(&retrieved, &selected), "Distinct wrappers for the same model rejected");
        check(!sameComponentModel(&retrieved, &assembly), "Part and assembly with the same name conflated");
        check(!sameComponentModel(&retrieved, &other), "Different model definitions conflated");
        check(!sameComponentModel(&retrieved, static_cast<ModelWrapper*>(nullptr)), "Null model accepted");
        const std::map<ComponentId, std::string> unique{{{40}, "BASE"}, {{75}, "ARM"}};
        auto names = resolveComponentNames(unique, {}, {{"BASE", "base_link"}});
        check(names.at({40}) == "base_link" && names.at({75}) == "ARM", "Legacy names changed");

        const std::map<ComponentId, std::string> repeated{{{40}, "LINK"}, {{75}, "LINK"}};
        names = resolveComponentNames(repeated, {}, {});
        check(names.at({40}) == "LINK__40" && names.at({75}) == "LINK__75", "Repeated parts collide");
        names = resolveComponentNames(repeated, {{{40}, "base"}, {{75}, "moving"}}, {});
        check(names.at({40}) == "base" && names.at({75}) == "moving", "Occurrence aliases ignored");

        const std::map<ComponentId, std::string> nested{{{40, 12}, "LINK"}, {{75, 12}, "LINK"}};
        names = resolveComponentNames(nested, {}, {});
        check(names.at({40, 12}) == "LINK__40_12" && names.at({75, 12}) == "LINK__75_12",
              "Repeated subassemblies lost their ancestor identity");
        check(componentIdString({1, 23}) != componentIdString({12, 3}), "Path serialization collides");

        rejects([&] { resolveComponentNames(repeated, {}, {{"LINK", "link"}}); }, "Ambiguous legacy rename accepted");
        rejects([&] { resolveComponentNames(repeated, {{{40}, "same"}, {{75}, "same"}}, {}); }, "Duplicate alias accepted");
        rejects([&] { resolveComponentNames(unique, {{{99}, "missing"}}, {}); }, "Unknown path accepted");
        rejects([&] { resolveComponentNames(unique, {{{40}, ""}}, {}); }, "Empty name accepted");
        rejects([&] { resolveComponentNames(repeated, {{{40}, "LINK__75"}}, {}); }, "Alias collides with generated name");
        auto collision = repeated;
        collision.emplace(ComponentId{90}, "LINK__40");
        rejects([&] { resolveComponentNames(collision, {}, {}); }, "CAD name collides with generated name");

        // Explicit aliases take precedence over a legacy entry for the repeated model.
        names = resolveComponentNames(repeated, {{{40}, "base"}, {{75}, "moving"}}, {{"LINK", "old"}});
        check(names.at({40}) == "base" && names.at({75}) == "moving", "Explicit aliases did not take precedence");
        std::cout << "Component identity tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
