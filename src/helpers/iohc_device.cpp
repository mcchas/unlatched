#include <iohc_device.h>
#include <numeric>

namespace IOHC {
//     return _iohcDevice;

bool iohcDevice::isFake(address nodeSrc, address nodeDst) {
    this->Fake = false;
    return this->Fake;
}

bool iohcDevice::isHome(address nodeSrc, address nodeDst) {
    this->Home = false;
    return this->Home;
}
}  // namespace IOHC
