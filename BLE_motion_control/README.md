#Robot met bluetooth besturen

De bedoeling hiervan was om te proberen de robot met bluetooth te kunnen aansturen. Er waren echter enkele problemen. 
De setpoint verzetten was geen goede manier om vooruit of achteruit te rijden. Dit komt omdat de robot niet getuned is voor deze setpoint en ook geen snelheidscontroller heeft.
Ook was de looptime opeens veel groter door het constant inlezen van de bluetooth waardoor de hoek minder nauwkeurig was en de PID loop zelf ook.
