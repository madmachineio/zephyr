# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_RPMSG_SERVICE rpmsg_service)
add_subdirectory_ifdef(CONFIG_RPMSG_MULTI_INSTANCE rpmsg_multi_instance)
add_subdirectory_ifdef(CONFIG_IPC_SERVICE ipc_service)
