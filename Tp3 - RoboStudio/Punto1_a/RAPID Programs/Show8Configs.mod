MODULE Module1
        CONST robtarget Pos_inicial:=[[-533.014588406,-47.835810924,828.73894736],[0.635481621,-0.251423899,-0.640712069,0.349910242],[-2,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget BrazoAdel_CodoPos_MunPos:=[[-533.014588406,-47.835810924,828.73894736],[0.635481621,-0.251423899,-0.640712069,0.349910242],[-2,-2,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget BrazoAdel_CodoPos_MunNeg:=[[-533.014588406,-47.835810924,828.73894736],[0.635481621,-0.251423899,-0.640712069,0.349910242],[-2,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget BrazoAdel_CodoNeg_MunPos:=[[-533.014588406,-47.835810924,828.73894736],[0.635481621,-0.251423899,-0.640712069,0.349910242],[-2,-2,0,3],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget BrazoAdel_CodoNeg_MunNeg:=[[-533.014588406,-47.835810924,828.73894736],[0.635481621,-0.251423899,-0.640712069,0.349910242],[-2,0,-2,2],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget BrazoAtras_CodoPos_MunPos:=[[-533.014588406,-47.835810924,828.73894736],[0.635481621,-0.251423899,-0.640712069,0.349910242],[0,0,0,7],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget BrazoAtras_CodoPos_MunNeg:=[[-533.014588406,-47.835810924,828.73894736],[0.635481621,-0.251423899,-0.640712069,0.349910242],[0,-2,-2,6],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget BrazoAtras_CodoNeg_MunPos:=[[-533.014588406,-47.835810924,828.73894736],[0.635481621,-0.251423899,-0.640712069,0.349910242],[0,0,0,5],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget BrazoAtras_CodoNeg_MunNeg:=[[-533.014588406,-47.835810924,828.73894736],[0.635481621,-0.251423899,-0.640712069,0.349910242],[0,-2,-2,4],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
!***********************************************************
    !
    ! Module:  Module1
    !
    ! Description:
    !   <Insert description here>
    !
    ! Author: Ramiro
    !
    ! Version: 1.0
    !
    !***********************************************************
    
    
    !***********************************************************
    !
    ! Procedure main
    !
    !   This is the entry point of your program
    !
    !***********************************************************
    PROC main()
        !Add your code here
    ENDPROC
    PROC Show8Configs()
        MoveJ Pos_inicial,v1000,z100,tool0\WObj:=wobj0;
        MoveJ BrazoAdel_CodoPos_MunPos,v1000,z100,tool0\WObj:=wobj0;
        MoveJ BrazoAdel_CodoPos_MunNeg,v1000,z100,tool0\WObj:=wobj0;
        MoveJ BrazoAdel_CodoNeg_MunPos,v1000,z100,tool0\WObj:=wobj0;
        MoveJ BrazoAdel_CodoNeg_MunNeg,v1000,z100,tool0\WObj:=wobj0;
        MoveJ BrazoAtras_CodoPos_MunPos,v1000,z100,tool0\WObj:=wobj0;
        MoveJ BrazoAtras_CodoPos_MunNeg,v1000,z100,tool0\WObj:=wobj0;
        MoveJ BrazoAtras_CodoNeg_MunPos,v1000,z100,tool0\WObj:=wobj0;
        MoveJ BrazoAtras_CodoNeg_MunNeg,v1000,z100,tool0\WObj:=wobj0;
    ENDPROC
ENDMODULE