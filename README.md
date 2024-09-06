한이음 2024년 ICT멘토링 <교통관제를 위한 AI 가변중앙분리대>

detect_custom.py : yolov7을 통해 이미지 분석을 수행하고 중앙분리대의 이동을 판단
union.py : detect_custom.py에서 판단한 데이터를 받아 STM32로의 시리얼 통신을 수행
