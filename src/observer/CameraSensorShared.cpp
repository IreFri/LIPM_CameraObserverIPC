#include <mc_rtc/logging.h>

#include <lipm_walking/observer/CameraSensorShared.h>
#include <signal.h>

namespace lipm_walking
{

bool PID::isAlive() const noexcept
{
  if(pid == -1)
  {
    return false;
  }
  return kill(pid, 0) == 0;
}

namespace
{

ipc::managed_shared_memory & get_shm()
{
  boost::interprocess::permissions  unrestricted_permissions;
  unrestricted_permissions.set_unrestricted();

  // Allocate enough space for 64 cameras and 1024 points per camera
  static ipc::managed_shared_memory shm(ipc::open_or_create,
                                        CAMERA_SENSOR_SHM_ID,
                                        64 * (sizeof(CameraSensorShared) + 1024 * sizeof(Eigen::Vector3d)),
                                        0,
                                        unrestricted_permissions);
  return shm;
}

} // namespace

CameraSensorShared::CameraSensorShared(CameraSensorShared::CtorKey)
: points(CameraSensorShared::v3d_allocator(get_shm().get_segment_manager())),
  ground_points(CameraSensorShared::v3d_allocator(get_shm().get_segment_manager())),
  aligned_points(CameraSensorShared::v3d_allocator(get_shm().get_segment_manager())),
  selected_points(CameraSensorShared::v3d_allocator(get_shm().get_segment_manager()))
{
}

CameraSensorShared * CameraSensorShared::get(const char * name)
{
  auto & shm = get_shm();
  auto * out = shm.find_or_construct<CameraSensorShared>(name)(CtorKey{});
  return out;
}

void CameraSensorShared::newClient(pid_t pid)
{
  if(server.isAlive())
  {
    ipc::scoped_lock<ipc::interprocess_mutex> lck(mtx);
    server_idle.wait(lck);
  }
  ipc::scoped_lock<ipc::interprocess_mutex> lck(mtx);
  resetConditions();
  client.pid = pid;
}

void CameraSensorShared::newServer(pid_t pid)
{
  if(client.isAlive())
  {
    ipc::scoped_lock<ipc::interprocess_mutex> lck(mtx);
    client_idle.wait(lck);
  }
  ipc::scoped_lock<ipc::interprocess_mutex> lck(mtx);
  resetConditions();
  server.pid = pid;
}

void CameraSensorShared::resetConditions()
{
  auto & shm = get_shm();
  auto reset_cv = [&shm](ipc::offset_ptr<ipc::interprocess_condition> & cv)
  { cv = shm.construct<ipc::interprocess_condition>(ipc::anonymous_instance)(); };
  reset_cv(data_ready);
  reset_cv(compute_ready);
  reset_cv(result_ready);
}

} // namespace lipm_walking
