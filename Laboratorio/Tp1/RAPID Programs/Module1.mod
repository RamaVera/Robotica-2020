MODULE Module1
    CONST pose Perno_Pose_Horizontal:= [[0,0,90],[1, 0, 0, 0]];
    CONST pose Perno_Pose_Vertical:= [[90,0,0],[0.7071068, 0,0.7071068, 0]];
    PERS tooldata Pinza:=[TRUE,[[0,0,137],[0.980785,0,0,-0.19509]],[0.5,[0,0,100],[1,0,0,0],0,0,0]];
    !PERS tooldata Pinza_Perno:=[TRUE,[[0,0,137],[0.980785,0,0,-0.19509]],[0.1,[0,0,100],[1,0,0,0],0,0,0]];
    PERS tooldata Pinza_Perno:=[TRUE,[[0,0,227],[0.980785,0,0,-0.19509]],[0.1,[0,0,100],[1,0,0,0],0,0,0]];
    PERS wobjdata wobjMesa:=[FALSE,TRUE,"",[[665,5,260],[0,0,0,1]],[[0,0,0],[1,0,0,0]]];
    CONST robtarget PosicionInicial:=[[600,0,600],[0.490392593,-0.168953224,0.849384987,-0.097545151],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget AgarrePernoVertical:=[[400,170,130],[0,-0.707106781,0.707106781,0],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget AgarrePernoHorizontal:=[[400,170,130],[0.5,0.5,-0.5,0.5],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget AgujeroMesa:=[[65,65,0],[0,0.707106781,0.707106781,0],[-1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
!***********************************************************
    !
    ! Modulo:  Module1
    !
    ! Descripcion:
    !   <Introduzca la descripcion aqui>
    !
    ! Autor: Ramiro
    !
    ! Versi?n: 1.0
    !
    !***********************************************************

    
    !***********************************************************
    !
    ! Procedimiento Main
    !
    !   Este es el punto de entrada de su programa
    !
    !***********************************************************
    CONST speeddata Velocidad:=v300;
    PROC main() 
 
! ***************************************************************************************
!                           Descomentar para usar FlexPendant
! ***************************************************************************************
!       VAR bool AgarradoHorizontal;
!       VAR bool AgarradoVertical; 
!        TPReadFK reg1, "Como debe agarrar el perno?", "Vertical", stEmpty, stEmpty, stEmpty, "Horizontal";
!        IF     reg1 = 1 THEN
!            AgarradoHorizontal:= FALSE ;
!            AgarradoVertical:= NOT AgarradoHorizontal;
!        ELSEIF reg1 = 5 THEN
!            AgarradoHorizontal:= TRUE ;
!            AgarradoVertical:= NOT AgarradoHorizontal;
!        ELSE
!        ENDIF
! ***************************************************************************************
!                           Descomentar para Debug sin FlexPendant
! *************************************************************************************** 
! Definir AgarradoHorizontal como TRUE o FALSE comentando y descomentando la instruccion
! correspondiente
        !CONST bool AgarradoHorizontal:= FALSE ;
        CONST bool AgarradoHorizontal:= TRUE;
        CONST bool AgarradoVertical:= NOT AgarradoHorizontal;
!***************************************************************************************
        
           
        MoversePosInicial;
        IF      AgarradoHorizontal  THEN  
            MoverseAlPernoHorizontalmente;
            AgarrarPernoHorizontalmente;
        ELSEIF  AgarradoVertical    THEN  
            MoverseAlPernoVerticalmente;
            AgarrarPernoVerticalmente;
        ELSE 
            EnviarMensajeError;
        ENDIF
        MoverseAlPunto;
        SoltarPerno;
        MoversePosInicial;
        
    ENDPROC
    
    !***********************************************************
    ! FUNCIONES INVOCADAS
    !***********************************************************
    !Descripcion: Mueve el robot a la posicion inicial
    !Precondicion: Debe estar referido al wobj0
    !Poscondicion: El robot queda en la posicion inicial
    PROC MoversePosInicial()
        MoveJ PosicionInicial,Velocidad,fine,Pinza\WObj:=wobj0;
    ENDPROC
    
    !Descripcion: Mueve el robot a la posicion de agarre del perno
    !para ser agarrado de forma perpendicular
    !Precondicion: Debe estar referido al wobj0
    !Poscondicion: El robot queda en la posicion [400,170,150] acorde
    !con 20 mm lejos sobre el eje z de la posicion de agarre del perno
    PROC MoverseAlPernoVerticalmente()
        VAR robtarget aux;
        !aux:= RelTool(PosicionInicial, -200, 0, 0);
        aux:= Offs(PosicionInicial, -200, 0, 0);
        MoveL aux,Velocidad,fine,Pinza\WObj:=wobj0;
        !aux:= RelTool(aux,0, 170, 0);
        aux:= Offs(aux,0, 170, 0);
        MoveL aux,Velocidad,fine,Pinza\WObj:=wobj0;
        !Se mueve la pinza 20mm alejado antes del agarre vertical
        MoveJ Offs(AgarrePernoVertical,0,0,20),Velocidad,fine,Pinza\WObj:=wobj0;
        
    ENDPROC
    
    !Descripcion: Mueve el robot a la posicion de agarre del perno
    !para ser agarrado de forma horizontal
    !Precondicion: Debe estar referido al wobj0
    !Poscondicion: El robot queda en la posicion [400,190,130] acorde
    !con 20 mm lejos sobre el eje y la posicion de agarre del perno
    PROC MoverseAlPernoHorizontalmente()
        VAR robtarget aux;
        !aux:= RelTool(PosicionInicial, -200, 0, 0);
        aux:= Offs(PosicionInicial, -200, 0, 0);
        MoveL aux,Velocidad,fine,Pinza\WObj:=wobj0;
        !aux:= RelTool(aux,0, 170, 0);
        aux:= Offs(aux,0, 170, 0);
        MoveL aux,Velocidad,fine,Pinza\WObj:=wobj0;
        ! Lo muevo 20 mm alejado de la posicion de agarre horizontal
        MoveJ Offs(AgarrePernoHorizontal,0,20,0),Velocidad,fine,Pinza\WObj:=wobj0;
    ENDPROC
    
    !Descripcion: Agarra el perno y lo une a la pinza verticalmente
    !Precondicion: Debe estar posicionado en la posicion de agarre
    !Poscondicion: El robot debe cambiar de punta 
    PROC AgarrarPernoVerticalmente()
        !Se agarra vericalmente
        MoveJ AgarrePernoVertical,Velocidad,fine,Pinza\WObj:=wobj0;
        SetDO DO_Pinza,1;
        Pinza_Perno.tframe:= PoseMult(Pinza.tframe,Perno_Pose_Vertical);
    ENDPROC
    
    !Descripcion: Agarra el perno y lo une a la pinza horizontalmente
    !Precondicion: Debe estar posicionado en la posicion de agarre
    !Poscondicion: El robot debe cambiar de punta 
    PROC AgarrarPernoHorizontalmente()
        ! Se agarra horizontalmente
        MoveJ AgarrePernoHorizontal,Velocidad,fine,Pinza\WObj:=wobj0;
        SetDO DO_Pinza,1;
        Pinza_Perno.tframe:= PoseMult(Pinza.tframe,Perno_Pose_Horizontal);
    ENDPROC
    
    !Descripcion: Suelta el perno y vuelve a definir a la pinza como punta
    !Precondicion: Se suelta donde el robot quedo anteriormente
    !Poscondicion: El robot debe cambiar de punta 
    PROC SoltarPerno()
        SetDO DO_Pinza,0;
    ENDPROC

    !Descripcion: Mueve la pinza junto al perno hacia el agujero de la mesa sin soltar el perno
    !Precondicion: Se debe haber definido previamente Pinza_Perno 
    !Poscondicion: 
    PROC MoverseAlPunto()
        MoveL Offs(AgarrePernoVertical,0,0,300),Velocidad,fine,Pinza\WObj:=wobj0;
        ConfJ \On;
        MoveJ Offs(AgujeroMesa,0,0,200),Velocidad,fine,Pinza_Perno\WObj:=wobjMesa;
        MoveL AgujeroMesa,Velocidad,fine,Pinza_Perno\WObj:=wobjMesa;  
    ENDPROC
    
    !Descripcion: Envia un mensaje de error
    !Precondicion: Debe ser llamado cuando exista un error
    !Poscondicion: 
    PROC EnviarMensajeError()
        ErrLog 100, ERRSTR_TASK, "Agarre No definido", ERRSTR_CONTEXT,ERRSTR_UNUSED, ERRSTR_UNUSED;
    ENDPROC
    PROC AgarrarPerno()
        SetDO DO_Pinza,1;
    ENDPROC
    
    
ENDMODULE