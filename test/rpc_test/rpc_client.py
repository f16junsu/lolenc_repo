import sipyco.pc_rpc as rpc

if __name__ == "__main__":
    client = rpc.Client("127.0.0.4", 8080,"adder")
    client.add(1, 2)
    client.print_value()