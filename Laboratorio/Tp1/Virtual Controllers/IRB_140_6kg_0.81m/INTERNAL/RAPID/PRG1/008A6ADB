MODULE Module1
    PERS tooldata Pinza:=[TRUE,[[0,0,137],[0.98078528,0,0,-0.195090322]],[0.5,[0,0,100],[1,0,0,0],0,0,0]];
    PERS wobjdata wobjMesa:=[FALSE,TRUE,"",[[665,5,260],[0,0,0,1]],[[0,0,0],[1,0,0,0]]];
    CONST robtarget PosicionInicial:=[[600,0,600],[0.490392593,-0.168953224,0.849384987,-0.097545151],[0,2,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Aux1:=[[400.000008995,170.000009452,600.000002468],[-0.000000008,0.707106992,-0.707106571,0.000000082],[0,1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget AgarrePerno:=[[400,170,130],[0,-0.707106781,0.707106781,0],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
!***********************************************************
    !
    ! M?dulo:  Module1
    !
    ! Descripci?n:
    !   <Introduzca la descripci?n aqu?>
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
    PROC main() 
        CONST bool AgarradoHorizontal:= FALSE;
        CONST bool AgarradoVertical:= NOT AgarradoHorizontal;
    
        MoversePosInicial;
        IF AgarradoHorizontal THEN MoverseAlPernoHorizontalmente;
        ELSEIF AgarradoVertical THEN MoverseAlPernoVerticalmente;
        ELSE ErrLog 100, ERRSTR_TASK, "Agarre No definido", ERRSTR_CONTEXT,ERRSTR_UNUSED, ERRSTR_UNUSED;
        ENDIF
 !       AgarrarPerno;
  !      MoverseAlPunto;
   !     SoltarPerno;
        
        MoversePosInicial;
        
    ENDPROC
    
    !***********************************************************
    ! FUNCIONES INVOCADAS
    !***********************************************************
    !Descripcion: Mueve el robot a la posicion inicial
    !Precondicion: Debe estar referido al wobj0
    !Poscondicion: El robot queda en la posicion inicial
    PROC MoversePosInicial()
        MoveJ PosicionInicial,v1000,fine,Pinza\WObj:=wobj0;
    ENDPROC
    
    !Descripcion: Mueve el robot a la posicion de agarre del perno
    !para ser agarrado de forma perpendicular
    !Precondicion: Debe estar referido al wobj0
    !Poscondicion: El robot queda en la posicion [400,170,130] acorde
    !con la posicion de agarre del perno
    PROC MoverseAlPernoVerticalmente()
        VAR robtarget aux;
        !aux:= RelTool(PosicionInicial, -200, 0, 0);
        aux:= Offs(PosicionInicial, -200, 0, 0);
        MoveL aux,v1000,fine,Pinza\WObj:=wobj0;
        !aux:= RelTool(aux,0, 170, 0);
        aux:= Offs(aux,0, 170, 0);
        MoveL aux,v1000,fine,Pinza\WObj:=wobj0;
        !Aux1 define una rotacion para poner la pinza de manera Vertical
        MoveJ Aux1,v1000,fine,Pinza\WObj:=wobj0;
        MoveL AgarrePerno,v10,fine,Pinza\WObj:=wobj0;
    ENDPROC
    
    !Descripcion: Mueve el robot a la posicion de agarre del perno
    !para ser agarrado de forma horizontal
    !Precondicion: Debe estar referido al wobj0
    !Poscondicion: El robot queda en la posicion [400,170,130] acorde
    !con la posicion de agarre del perno
    PROC MoverseAlPernoHorizontalmente()
    ENDPROC
    
    !Descripcion: Agarra el perno y lo une a la pinza
    !Precondicion: Debe estar posicionado en la posicion de agarre
    !Poscondicion: El robot debe cambiar de punta 
    PROC AgarrarPerno()
        SetDO DO_Pinza,1;
        
    ENDPROC
    
    !Descripcion: Suelta el perno y vuelve a definir a la pinza como punta
    !Precondicion: Se suelta donde el robot quedo anteriormente
    !Poscondicion: El robot debe cambiar de punta 
    PROC SoltarPerno()
        SetDO DO_Pinza,0;
        
    ENDPROC

    !Descripcion: Suelta el perno y vuelve a definir a la pinza como punta
    !Precondicion: Se suelta donde el robot quedo anteriormente
    !Poscondicion: El robot debe cambiar de punta 
    PROC MoverseAlPunto()
       
    ENDPROC
    
    
    
ENDMODULE