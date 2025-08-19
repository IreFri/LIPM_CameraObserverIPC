#include <lipm_walking/observer/CameraSensorShared.h>

/** This should be called whenever:
 * - CameraSensorShared is changed
 * - Servers or client seem stuck
 */

int main()
{
  ipc::shared_memory_object::remove(lipm_walking::CAMERA_SENSOR_SHM_ID);
  return 0;
}
