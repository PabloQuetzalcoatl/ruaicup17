cd F:\AICup\2017\local-runner\
	
start /b F:\AICup\2017\local-runner\local-runner-sync.bat

timeout /t  3


echo "start strategy"
python36 F:\AICup\2017\strategy\Runner.py