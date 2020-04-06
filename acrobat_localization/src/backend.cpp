#include <acrobat_localization/backend.hpp>

namespace acrobat::localization {

Backend::SharedPtr Backend::create() { return nullptr; }

void Backend::run() {}

} // namespace acrobat::localization
