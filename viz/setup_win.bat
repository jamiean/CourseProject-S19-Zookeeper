@rem Usage:
@rem    setup_win
@rem
@echo.
@echo FIRST YOU MUST DOWNLOAD AND INSTALL THE FOLLOWING:
@echo.
@echo   1) https://www.python.org/ftp/python/2.7.8/python-2.7.8.msi
@echo   2) https://bootstrap.pypa.io/get-pip.py
@echo   3) http://www.microsoft.com/en-us/download/confirmation.aspx?id=44266
@echo.
@echo Do each of the above steps, then press a key.
@echo.
@pause
@echo Installing python-dateutil
\Python27\Scripts\pip install python-dateutil
@echo.
@pause
@echo Installing pyparsing
\Python27\Scripts\pip install pyparsing
@echo.
@pause
@echo Installing configobj
\Python27\Scripts\pip install configobj
@echo.
@pause
@echo Installing requests
\Python27\Scripts\pip install requests
@echo.
@pause
@echo Installing numpy (this may take a while)
\Python27\Scripts\pip install numpy
@echo.
@pause
@echo Installing networkx
\Python27\Scripts\pip install networkx
@echo.
@pause
@echo Installing matplotlib
\Python27\Scripts\pip install matplotlib
@echo.
@pause
@echo Installing mpltools
\Python27\Scripts\pip install mpltools
