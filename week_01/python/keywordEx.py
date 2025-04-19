import keyword  # 'keyword' 모듈을 가져옵니다. 이 모듈은 파이썬에서 예약어 목록을 제공하는 기능을 합니다.

def main():  # main() 함수 정의 시작
    print(keyword.kwlist)  # 'kwlist' 속성을 사용하여 파이썬 예약어들의 목록을 출력합니다.

if __name__ == "__main__":  # 이 파일이 직접 실행되었을 때만 아래 코드를 실행하도록 합니다.
    main()  # main() 함수를 호출하여 프로그램을 실행합니다.
