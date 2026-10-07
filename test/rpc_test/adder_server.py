from sipyco.pc_rpc import simple_server_loop

class Adder:
    def __init__(self):
        self.value = 0

    def add(self, a : float, b : float)-> float:
        self.value = a + b + self.value
        print(f"add {self.value}")
        return a + b

    def get_value(self) -> float:
        print(f"get value {self.value}")
        return self.value

    def set_value(self, value : float) -> None:
        print(f"set value {self.value}")
        self.value = value
        print(f"set value {self.value}")

    def print_value(self) -> None:
        print(f"print value {self.value}")

def main():
    simple_server_loop({"adder" : Adder()}, "127.0.0.4", 8080)

if __name__ == "__main__":
    main()
