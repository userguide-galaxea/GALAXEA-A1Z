"""Cross-platform transport tests for the local a1zctl server."""

import unittest
from unittest.mock import Mock, patch

from a1z.robots import server


class ServerTransportTests(unittest.TestCase):
    def test_tcp_endpoint_is_loopback_only(self):
        with patch.object(server, "_use_unix_socket", return_value=False):
            self.assertEqual(
                server.endpoint_label(),
                f"tcp://127.0.0.1:{server.TCP_PORT}",
            )

    def test_client_uses_tcp_when_unix_sockets_are_unavailable(self):
        sock = Mock()
        with (
            patch.object(server, "_use_unix_socket", return_value=False),
            patch.object(server.socket, "socket", return_value=sock) as create,
        ):
            result = server.connect_to_server(timeout=3.0)

        self.assertIs(result, sock)
        create.assert_called_once_with(server.socket.AF_INET, server.socket.SOCK_STREAM)
        sock.settimeout.assert_called_once_with(3.0)
        sock.connect.assert_called_once_with(("127.0.0.1", server.TCP_PORT))

    def test_client_closes_socket_when_connect_fails(self):
        sock = Mock()
        sock.connect.side_effect = ConnectionRefusedError
        with (
            patch.object(server, "_use_unix_socket", return_value=False),
            patch.object(server.socket, "socket", return_value=sock),
            self.assertRaises(ConnectionRefusedError),
        ):
            server.connect_to_server()

        sock.close.assert_called_once_with()

    def test_server_binds_tcp_loopback_when_unix_sockets_are_unavailable(self):
        sock = Mock()
        robot_server = server.RobotServer(Mock(), with_gripper=False)
        robot_server._shutdown.set()
        with (
            patch.object(server, "_use_unix_socket", return_value=False),
            patch.object(server.socket, "socket", return_value=sock) as create,
            patch("builtins.print"),
        ):
            robot_server.run()

        create.assert_called_once_with(server.socket.AF_INET, server.socket.SOCK_STREAM)
        sock.bind.assert_called_once_with(("127.0.0.1", server.TCP_PORT))
        sock.listen.assert_called_once_with(8)
        sock.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
