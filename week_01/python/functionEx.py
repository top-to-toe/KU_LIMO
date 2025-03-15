def print_3_time():
    for i in range(3):
        print("안녕하세요")
    a = 100
    return "ok", 1, "complete", a


def main():
    re = print_3_time()
    print(re)


if __name__ == "__main__":
    main()
