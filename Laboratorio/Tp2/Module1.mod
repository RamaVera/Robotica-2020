MODULE Module1
    PERS tooldata ventosa_tool         :=[TRUE,[[0,0,200],[1,0,0,0]],[1,[0,0,1],[1,0,0,0],0,0,0]];
    PERS wobjdata pistaCajas           :=[FALSE,TRUE,"",[[-200,1500,770],[0,0,1,0]],[[0,0,0],[1,0,0,0]]];
    PERS wobjdata pistaPallets         :=[FALSE,TRUE,"",[[1000,-1425,579],[0,0,1,0]],[[0,0,0],[1,0,0,0]]];
    CONST robtarget lugarEspera:=[[0,0,-1000],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget tomaCaja_10:=[[0,400,-500],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget puntoIntermedio_10:=[[1500,1500,2000],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget superficiePallet_10:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    CONST num anchoPallet := 800; ![mm]
    CONST num largoPallet := 1200;![mm]
    CONST num anchoCaja   := 400; ![mm]
    CONST num largoCaja   := 800;![mm]
    CONST num altoCaja    := 500; ![mm]
    CONST num nCajaMax    := 12;    
    
!Numero de cajas por pallet
!***********************************************************
    !
    ! Módulo:  Module1
    !
    ! Descripción:
    !   Simulación de una planta de paletizado
    !
    ! Autor: Eichenbaum Daniel
    !        Vera Ramiro
    !
    ! cuatrimestre: 2do
    ! Año: 2020
    ! Versión: 1.0
    !
    !***********************************************************
    PROC main()
        !Añada aquí su código
        VAR robtarget pos;!posicion del robtarget actual
        SetDO do_pick,0;
        
        FOR nCaja FROM 0 TO nCajaMax-1 DO
            !nCaja = numero de cajas ya colocadas  
            esperarCaja;
            tomarSiguienteCaja;
            pos := calcularSiguientePosicion(nCaja);!Calcula posicion de la torre
            colocarCaja(pos);
        ENDFOR
        esperarCaja;
    ENDPROC
    
    PROC esperarCaja()
        MoveJ lugarEspera,v1000,fine,ventosa_tool\WObj:=pistaCajas;
        WaitDI cajaLista,1;
        !Esperar señal del sensor
    ENDPROC
    PROC tomarSiguienteCaja()
        MoveJ tomaCaja_10,v1000,fine,ventosa_tool\WObj:=pistaCajas;
        SetDO do_pick, 1;
        WaitTime 0.2;
        MoveJ puntoIntermedio_10,v1000,z100,ventosa_tool\WObj:=wobj0;
    ENDPROC 
    
    FUNC robtarget calcularSiguientePosicion(num nCaja)
        VAR robtarget pResult;
        VAR num nCapa; !Numero de la capa
        nCapa := nCaja DIV 3 + 1; !(division entera)
        pResult := superficiePallet_10;!copio el origen del pallet
        TEST nCaja MOD 6 + 1
        CASE 1:
            pResult.trans.x := -largoPallet+anchoCaja/2;
            pResult.trans.y := anchoPallet /2;
            pResult.trans.z := -altoCaja*nCapa;
        CASE 2:
            pResult.trans.x := -largoCaja/2;
            pResult.trans.y :=  anchoCaja /2;
            pResult.trans.z := -altoCaja*nCapa;
            pResult.rot     := OrientZYX(90,0,0);
        CASE 3:
            pResult.trans.x := -largoCaja/2;
            pResult.trans.y :=  anchoPallet-anchoCaja /2;
            pResult.trans.z := -altoCaja*nCapa;
            pResult.rot     := OrientZYX(-90,0,0);
        CASE 4:
            pResult.trans.x := -largoPallet+largoCaja/2;
            pResult.trans.y :=  anchoCaja /2;
            pResult.trans.z := -altoCaja*nCapa;
            pResult.rot     := OrientZYX(90,0,0);
        CASE 5:
            pResult.trans.x := -largoPallet+largoCaja/2;
            pResult.trans.y := anchoPallet-anchoCaja/2;
            pResult.trans.z := -altoCaja*nCapa;
            pResult.rot     := OrientZYX(-90,0,0);
        CASE 6:
            pResult.trans.x := -anchoCaja/2;
            pResult.trans.y :=  anchoPallet/2;
            pResult.trans.z := -altoCaja*nCapa;
            pResult.rot     := OrientZYX(180,0,0);
        ENDTEST
        RETURN pResult;
    ENDFUNC   
    
    PROC colocarCaja(robtarget posFinal)
        !Sin chocar con las cajas ya puestas
        MoveJ Offs(posFinal,0,1500,-500), v1000, z100, ventosa_tool\WObj:=pistaPallets;!Antes de llegar al pallet 1.2m tiene que estar bien orientado
        !MoveJ Offs(posFinal,0,500,-500), v1000, fine, ventosa_tool\WObj:=pistaPallets;
        MoveL posFinal, v1000, fine, ventosa_tool\WObj:=pistaPallets;
        SetDO do_pick, 0;
        WaitTime 0.2;
    ENDPROC
ENDMODULE