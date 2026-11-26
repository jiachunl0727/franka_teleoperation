"""Validate runtime profiles using real libnm, without D-Bus or NIC changes.

Run with /usr/bin/python3 on the commissioning host (libnm 1.36).
"""

import unittest

try:
    import gi
    gi.require_version('NM', '1.0')
    from gi.repository import NM, GLib
except (ImportError, ValueError):
    NM = GLib = None

from direct_robot_network import LAYOUTS, profile_keyfile


@unittest.skipIf(NM is None, 'Use host /usr/bin/python3 with libnm introspection installed')
class NativeProfileTests(unittest.TestCase):
    def test_both_profiles_verify_with_fixed_uuid_and_correct_routes(self):
        for layout, links in LAYOUTS.items():
          for link in links:
            with self.subTest(layout=layout, side=link['side']):
                data=profile_keyfile(link)
                keyfile=GLib.KeyFile()
                keyfile.load_from_data(data,len(data),GLib.KeyFileFlags.NONE)
                connection=NM.keyfile_read(keyfile,'/run/NetworkManager/system-connections',
                                           NM.KeyfileHandlerFlags.NONE,None,None)
                self.assertTrue(connection.verify())
                self.assertEqual(connection.get_uuid(),link['uuid'])
                setting=connection.get_setting_connection()
                self.assertFalse(setting.get_autoconnect())
                self.assertEqual(setting.get_interface_name(),link['dev'])
                ip=connection.get_setting_ip4_config()
                self.assertEqual(ip.get_address(0).get_address(),link['source'])
                self.assertEqual(ip.get_address(0).get_prefix(),32)
                self.assertEqual(ip.get_num_routes(),1)
                self.assertEqual(ip.get_route(0).get_dest(),link['robot'])
                self.assertEqual(ip.get_route(0).get_prefix(),32)
                self.assertFalse(ip.get_gateway())
                self.assertTrue(ip.get_never_default())


if __name__=='__main__':
    unittest.main()
