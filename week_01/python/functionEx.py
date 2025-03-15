# 'print_3_time' 함수 정의: "안녕하세요"를 3번 출력하고 여러 값을 반환합니다.
def print_3_time():
    for i in range(3):  # range(3)은 0, 1, 2 3번 반복합니다.
        print("안녕하세요")  # "안녕하세요"를 출력합니다.
    
    a = 100  # 변수 'a'에 100을 저장합니다.
    return "ok", 1, "complete", a  # 여러 개의 값을 반환합니다. 반환값은 튜플로 묶입니다.

# 'print_n_time' 함수 정의: 'n'번 만큼 "안녕하세요"를 출력합니다.
def print_n_time(n):
    for i in range(n):  # n번 반복합니다. n은 매개변수로 전달됩니다.
        print("안녕하세요")  # "안녕하세요"를 출력합니다.

# main 함수 정의: 전체 프로그램의 흐름을 제어합니다.
def main():
    re = print_3_time()  # 'print_3_time' 함수를 호출하여 반환된 값을 're' 변수에 저장합니다.
    print_n_time(5)  # 'print_n_time' 함수에 5를 인수로 전달하여 "안녕하세요"를 5번 출력합니다.
    print(re)  # 're'에 저장된 반환값을 출력합니다. ('print_3_time' 함수에서 반환된 값)

# 이 파일이 직접 실행될 때만 main()을 호출하도록 합니다.
if __name__ == "__main__":  # __name__이 "__main__"일 때, 즉 이 파일이 직접 실행될 때만 아래 코드를 실행합니다.
    main()  # main 함수를 호출합니다.
