"""Provides a ctrlX Data Layer node to store the received ROS 2 value."""

# SPDX-FileCopyrightText: Bosch Rexroth AG
#
# SPDX-License-Identifier: MIT

import ctrlxdatalayer
from ctrlxdatalayer.provider import Provider
from ctrlxdatalayer.provider_node import ProviderNode, ProviderNodeCallbacks, NodeCallback
from ctrlxdatalayer.variant import Result, Variant


class CtrlXProviderNode:
    """
    Class CtrlXProviderNode.

    Provides a ctrlX Data Layer node and handles all events via callback functions.
    """

    def __init__(self,
                 datalayer_provider: Provider,
                 address: str,
                 variant: Variant):
        """Initialize the instance."""
        self.cbs = ProviderNodeCallbacks(
            self.__on_create,
            self.__on_remove,
            self.__on_browse,
            self.__on_read,
            self.__on_write,
            self.__on_metadata
        )

        self.providerNode = ProviderNode(self.cbs)

        self.provider = datalayer_provider
        self.address = address
        self.variant = variant

    def register_node(self):
        """Register the node."""
        return self.provider.register_node(self.address, self.providerNode)

    def unregister_node(self):
        """Unregister the node."""
        self.provider.unregister_node(self.address)

    def set_value(self, value: Variant):
        """Set the value."""
        self.variant = value

    def __on_create(self, userdata: ctrlxdatalayer.clib.userData_c_void_p, 
                    address: str, data: Variant, cb: NodeCallback):
        """Handle on_create event."""
        print('__on_create()', 'address:', address, 'userdata:', userdata)
        cb(Result.OK, data)

    def __on_remove(self, userdata: ctrlxdatalayer.clib.userData_c_void_p,
                    address: str, cb: NodeCallback):
        """Handle on_remove event."""
        print('__on_remove()', 'address:', address, 'userdata:', userdata)
        cb(Result.UNSUPPORTED, None)

    def __on_browse(self,
                    userdata: ctrlxdatalayer.clib.userData_c_void_p,
                    address: str,
                    cb: NodeCallback):
        """Handle on_browse event."""
        variant = Variant()
        variant.set_array_string([])
        cb(Result.OK, variant)

    def __on_read(self, userdata: ctrlxdatalayer.clib.userData_c_void_p,
                  address: str, data: Variant, cb: NodeCallback):
        """Handle on_read event."""
        variant = self.variant
        cb(Result.OK, variant)

    def __on_write(self, userdata: ctrlxdatalayer.clib.userData_c_void_p,
                   address: str, data: Variant, cb: NodeCallback):
        """Handle on_write event."""
        cb(Result.UNSUPPORTED, None)

    def __on_metadata(self, userdata: ctrlxdatalayer.clib.userData_c_void_p,
                    address: str,
                    cb: NodeCallback):
        """Handle on_metadata event."""
        cb(Result.FAILED, None)
