@echo off
cd /d %~dp0..
javac -cp "src\java\viewer\lib\jSerialComm-2.11.2.jar" src\java\viewer\Surface.java
java -cp "src\java\viewer;src\java\viewer\lib\jSerialComm-2.11.2.jar" Surface
