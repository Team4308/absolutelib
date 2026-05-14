./gradlew :coprocessor:shadowJar

$env:IS_SIMULATION='true'; $env:HTTP_PORT='5805'; C:\Users\Public\wpilib\2026\jdk\bin\java "-Djava.library.path=.\coprocessor\build\natives\windows\x86-64" -jar .\coprocessor\build\libs\coprocessor.jar