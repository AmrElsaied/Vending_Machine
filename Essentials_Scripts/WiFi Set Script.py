#!/usr/bin/env python3
import socket
import argparse
import sys


def send_tcp_data(host: str, port: int, data: str = None, timeout: float = 10.0) -> None:
    print(f"Connecting to {host}:{port}...")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)

        try:
            sock.connect((host, port))
            print(f"Connected to {host}:{port}")
        except socket.timeout:
            print(f"Error: Connection to {host}:{port} timed out.")
            sys.exit(1)
        except ConnectionRefusedError:
            print(f"Error: Connection refused by {host}:{port}.")
            sys.exit(1)
        except OSError as e:
            print(f"Error: {e}")
            sys.exit(1)

        # Prompt for data after connection is established
        if data is None:
            data = input("Data to send: ")

        # Encode and send data
        payload = data.encode("utf-8")
        try:
            sock.sendall(payload)
            print(f"Sent {len(payload)} byte(s): {data!r}")
        except socket.error as e:
            print(f"Send error: {e}")
            sys.exit(1)

        # Optionally receive a response
        try:
            response = sock.recv(4096)
            if response:
                print(f"Response ({len(response)} byte(s)): {response.decode('utf-8', errors='replace')!r}")
            else:
                print("Connection closed by remote host (no response).")
        except socket.timeout:
            print("No response received within timeout window.")
        except socket.error as e:
            print(f"Receive error: {e}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Open a TCP connection and send data.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python tcp_client.py 192.168.1.10 8080 "Hello, server!"
  python tcp_client.py example.com 443 "GET / HTTP/1.0\\r\\n\\r\\n" --timeout 5
        """,
    )
    parser.add_argument("host", nargs="?", help="Target IP address or hostname")
    parser.add_argument("port", nargs="?", type=int, help="Target TCP port (1–65535)")
    parser.add_argument("data", nargs="?", help="Data string to send over the connection")
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        metavar="SECONDS",
        help="Socket timeout in seconds (default: 10)",
    )

    args = parser.parse_args()

    if args.host is None:
        args.host = input("Host (IP or hostname): ").strip()
    if args.port is None:
        args.port = int(input("Port (1-65535): ").strip())

    if not (1 <= args.port <= 65535):
        parser.error(f"Port must be between 1 and 65535, got {args.port}")

    send_tcp_data(args.host, args.port, args.data, args.timeout)


if __name__ == "__main__":
    main()