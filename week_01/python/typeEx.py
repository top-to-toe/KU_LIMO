def main():  # main() 함수 정의 시작
    abc = int()  # abc를 int()로 초기화합니다. 기본적으로 abc는 0이 됩니다.
    abc = 4  # abc는 4로 변경됩니다. 이제 abc는 int 클래스의 객체입니다.
    print(abc, type(abc))  # abc의 값(4)과 abc의 타입(int)을 출력합니다.
    
    abc = 4.5  # abc는 실수 4.5로 변경됩니다.
    print(abc, type(abc))  # abc의 값(4.5)과 abc의 타입(float)을 출력합니다.
    
    abc = "this is python"  # abc는 문자열 "this is python"으로 변경됩니다.
    print(abc, type(abc))  # abc의 값("this is python")과 abc의 타입(str)을 출력합니다.

    # format -- f-string 사용 방법
    abc = "fstring"  # abc 변수에 "fstring"이라는 문자열을 할당합니다.
    number = 3.141592  # number 변수에 원주율 3.141592를 할당합니다.
    print(f"string string {abc} pi : {number:.3}")  # f-string을 사용하여 문자열을 출력합니다. 변수 abc와 number의 값을 삽입합니다. 
    # number는 소수점 3자리까지 출력됩니다.

if __name__ == "__main__":  # 이 파일이 직접 실행되었을 때만 아래 코드를 실행하도록 합니다.
    main()  # main() 함수를 호출하여 프로그램을 실행합니다.
