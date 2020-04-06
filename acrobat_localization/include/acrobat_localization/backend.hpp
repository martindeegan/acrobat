#pragma once

#include <memory>

namespace acrobat::localization {

class Backend {
  public:
    using SharedPtr = std::shared_ptr<Backend>;

    static SharedPtr create();

    virtual void run();

  private:
    Backend() = default;
};

} // namespace acrobat::localization