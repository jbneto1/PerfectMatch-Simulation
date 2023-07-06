unit laserloc;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, LResources, Forms, Controls, Graphics, Dialogs,
  StdCtrls, ComCtrls, IniPropStorage, ExtCtrls, lNetComponents, lNet, TAGraph,
  TASeries, Rlan, math, LclIntf, epiktimer, SynEdit, utils, BGRABitmap,
  BGRABitmapTypes, types;

type
  TLaserPoint = record
    d, angle: double;
    x, y: double;
    std: double;
  end;

  TLaserPoints = array of TLaserPoint;

  TPose = record
    x, y, theta, err: double;
  end;

  { TFLaserLoc }

  TFLaserLoc = class(TForm)
    BLogClear: TButton;
    BRobotPosSet: TButton;
    BSettingsSet: TButton;
    BLogSave: TButton;
    BVLogProcStart: TButton;
    BVLogSave: TButton;
    BVLogClear: TButton;
    BVLogLoad: TButton;
    CBShowSize: TCheckBox;
    ChartXY: TChart;
    CBInvertedLaser: TCheckBox;
    CBShowLoc: TCheckBox;
    CBSendLock: TCheckBox;
    CBAccumulation: TCheckBox;
    CBVlogActive: TCheckBox;
    CBLaserCalibration: TCheckBox;
    CBStdevComp: TCheckBox;
    CSLaserXY: TLineSeries;
    EditRobotThetaVloc: TEdit;
    EditRobotThetaErr: TEdit;
    EditRobotXVloc: TEdit;
    EditRobotXErr: TEdit;
    EditRobotYVLoc: TEdit;
    EditRobotYErr: TEdit;
    EditVLogNumLines: TEdit;
    EditLaserC1: TEdit;
    EditLaserC2: TEdit;
    EditLaserC3: TEdit;
    EditVLogFileName: TEdit;
    EditLogFile: TEdit;
    EditLaserXOffset: TEdit;
    EditNumAccPoints: TEdit;
    EditSendLockIP: TEdit;
    EditMaxIters: TEdit;
    EditFitError: TEdit;
    EditStepScale: TEdit;
    EditCerr: TEdit;
    EditTime: TEdit;
    EditLaserAngleOffset: TEdit;
    EditRobotThetaSet: TEdit;
    EditRobotX: TEdit;
    EditRobotXSet: TEdit;
    EditRobotY: TEdit;
    EditRobotTheta: TEdit;
    EditRobotYSet: TEdit;
    IniPropStorage: TIniPropStorage;
    Label1: TLabel;
    Label10: TLabel;
    Label11: TLabel;
    Label12: TLabel;
    Label13: TLabel;
    Label14: TLabel;
    Label15: TLabel;
    Label16: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    PaintBoxVLog: TPaintBox;
    RGLasers: TRadioGroup;
    TabLog: TTabSheet;
    TimerVLog: TTimer;
    UDPSend: TLUDPComponent;
    PageControl: TPageControl;
    PaintBox: TPaintBox;
    TabLaser: TTabSheet;
    TabLaserXY: TTabSheet;
    TabDebug: TTabSheet;
    TabRadial: TTabSheet;
    TabSettings: TTabSheet;
    UDP: TLUDPComponent;
    Memo: TMemo;
    Chart: TChart;
    CSLaser: TLineSeries;
    procedure BLogClearClick(Sender: TObject);
    procedure BLogSaveClick(Sender: TObject);
    procedure BRobotPosSetClick(Sender: TObject);
    procedure BSettingsSetClick(Sender: TObject);
    procedure BVLogClearClick(Sender: TObject);
    procedure BVLogLoadClick(Sender: TObject);
    procedure BVLogProcStartClick(Sender: TObject);
    procedure BVLogSaveClick(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure PaintBoxVLogPaint(Sender: TObject);
    procedure TimerVLogTimer(Sender: TObject);
    procedure UDPReceive(aSocket: TLSocket);
  private
    procedure ProcessAccumulation;
    procedure ProcessVLog;
    { private declarations }
  public
    NetInBuf: TUDPBuffer;
    LaserPoints, TmpLaserPoints: TLaserPoints;
    NumAccPoints: integer;
    tcount: integer;
    RealRobotPose, OdoRobotPose: TPose;
    odo1, odo2, RealRobotV, RealRobotW, dt: double;

    PerformanceCounter: int64;
    PerformanceFrequency: int64;
    SendLockPort: integer;

    ENC1, ENC2, ENC_count: integer;
    VLocRobotPose: TPose;               // Visual (external) Localization
    VLoc_count, Laser_count: integer;   // Visual (external) Localization Age

    LogFile, VLogFile: TStringList;
    VLogImage: TBGRABitmap;
    VLogLineNum: integer;

    RobotPoseMSE, RobotPoseMaxE: TPose;

    procedure SendLoc(ToIP: string; port: integer; Loc: TPose);
  end;

var
  FLaserLoc: TFLaserLoc;

  laserOffsetX: double;
  laserFirstIdx, laserLastIdx: integer;
  LaserAngleOffset: double;
  LaserAngleK1, LaserAngleK2: double;
  LaserDistanceC1, LaserDistanceC2, LaserDistanceC3: double;

  MaxIters: integer;
  StepScale: double;
  NetInBuf, NetOutBuf: TUDPBuffer;

  ET: TEpikTimer;

  GeMeCount, GeMeIdx: integer;


procedure debug(mess: string);


implementation

uses paint;

{ TFLaserLoc }

procedure debug(mess: string);
begin
  with FLaserLoc do begin
    Memo.Lines.Add(mess);
    while Memo.Lines.Count > 500 do begin
      Memo.Lines.Delete(0);
    end;
  end;
end;



procedure TFLaserLoc.FormShow(Sender: TObject);
begin
  FPaint.show;
  BSettingsSetClick(Sender);
  //UDP.Connect('127.0.0.1',9876)
  UDP.Listen(9876);
  SendLockPort := 9951;
  UDPSend.Connect('127.0.0.1', SendLockPort);
end;

procedure TFLaserLoc.PaintBoxVLogPaint(Sender: TObject);
var i, lc, x, y: integer;
    textsize: TSize;
    pix: TBGRAPixel;
begin
  pix := BGRA(255, 255, 255);
  if (VLogImage.Height <> PaintBoxVLog.Height) or
     (VLogImage.Width <> PaintBoxVLog.Width) then begin
    VLogImage.SetSize(PaintBoxVLog.Width, PaintBoxVLog.Height);
  end;
  VLogImage.FillRect(0, 0, VLogImage.Width, VLogImage.Height, pix, dmSet);
  VLogImage.FontHeight := 12;
  textsize := VLogImage.TextSize('W');
  //VLogImage.ScanAtInteger(0, textsize.cy);
  //VLogImage.ScanPutPixels( )
  {lc := VLogFile.Count - 1;
  y := VLogImage.Height - textsize.cy;
  x := 0;
  while (y > 0) do begin
    dec(lc);
    if lc < 0 then break;
    VLogImage.TextOut(x, y, VLogFile[lc], clBlack);
    dec(y, textsize.cy);
  end;}

  //VLogImage.TextOut(0, 0, 'hello', clBlack);

  y := 0;
  x := 0;
  for i := 0 to VLogFile.Count - 1 do begin
    VLogImage.TextOut(x, y, VLogFile.Strings[i], clBlack);
    inc(y, textsize.cy);
    if y > VLogImage.Height then break;

  end;


  VLogImage.Draw(PaintBoxVLog.Canvas, 0, 0, True)
end;

procedure TFLaserLoc.TimerVLogTimer(Sender: TObject);
begin
  if (VLogLineNum < 0) or (VLogLineNum >= VLogFile.Count) then exit;
  //UDPReceive(nil);
  EditVLogNumLines.Text := format('%d, %d', [VLogLineNum, VLogFile.Count]);
  ProcessVlog();

  inc(VLogLineNum);
  if VLogLineNum >= VLogFile.Count then VLogLineNum := -1;
  TimerVLog.Enabled := (VLogLineNum >= 0);
end;

procedure TFLaserLoc.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  ET.Stop;
  FPaint.Close;
end;

procedure TFLaserLoc.FormCreate(Sender: TObject);
begin
  ET := TEpikTimer.Create(Application);
  dt := 0.04;
  LogFile := TStringList.Create;
  VLogFile := TStringList.Create;
  VLogImage := TBGRABitmap.Create(PaintBoxVLog.Width, PaintBoxVLog.Height);
  VLogLineNum := -1;
end;

procedure TFLaserLoc.FormDestroy(Sender: TObject);
begin
  VLogImage.Free;
  VLogFile.Free;
  LogFile.Free;
  //ET.free;
end;

procedure TFLaserLoc.BSettingsSetClick(Sender: TObject);
var i: integer;
begin
  LaserAngleOffset := degtorad(StrToFloatDef(EditLaserAngleOffset.Text, 0));
  laserOffsetX := StrToFloat(EditLaserXOffset.text);

  LaserDistanceC1 :=  StrToFloat(EditLaserC1.text);
  LaserDistanceC2 :=  StrToFloat(EditLaserC2.text);
  LaserDistanceC3 :=  StrToFloat(EditLaserC3.text);

  MaxIters := StrToIntDef(EditMaxIters.Text, 10);
  StepScale := StrToFloatDef(EditStepScale.Text, 1e-2);
  c_err := StrToFloatDef(EditCerr.Text, 100);

  NumAccPoints := StrToIntDef(EditNumAccPoints.Text, 100);
  SetLength(LaserPoints, NumAccPoints);
  for i := 0 to Length(LaserPoints) - 1 do begin
    with LaserPoints[i] do  begin
      d := 0;
      x := 0;
      y := 0;
    end;
  end;


  with FPaint do
    FPaint. CalcGradMap(GradXMap, GradYMap, DistMap);
end;

procedure TFLaserLoc.BVLogClearClick(Sender: TObject);
begin
   VLogFile.Clear;
end;

procedure TFLaserLoc.BVLogLoadClick(Sender: TObject);
begin
  if FileExistsUTF8(EditVLogFileName.Text) then
    VLogFile.LoadFromFile(EditVLogFileName.Text);
  EditVLogNumLines.Text := IntToStr(VLogFile.Count);
  PaintBoxVLog.Refresh;
end;

procedure TFLaserLoc.BVLogProcStartClick(Sender: TObject);
begin
  VLogLineNum := min(0, VLogFile.Count - 1);
  //if VLogLineNum >= 0 then TimerVLog.Enabled := true;
  TimerVLog.Enabled := (VLogLineNum >= 0);

  with RobotPoseMSE do begin
    x := 0;
    y := 0;
    theta := 0;
  end;
  with RobotPoseMaxE do begin
    x := 0;
    y := 0;
    theta := 0;
  end;

end;

procedure TFLaserLoc.BVLogSaveClick(Sender: TObject);
begin
  if FileExistsUTF8(EditVLogFileName.Text) then DeleteFileUTF8(EditVLogFileName.Text);
  VLogFile.SaveToFile(EditVLogFileName.Text);
end;

procedure TFLaserLoc.BRobotPosSetClick(Sender: TObject);
begin
  RobotPose.x := StrToFloat(EditRobotXSet.Text);
  RobotPose.y := StrToFloat(EditRobotYSet.Text);
  RobotPose.theta := degtorad(StrToFloat(EditRobotThetaSet.Text));
  FPaint.FormPaint(FPaint);
end;

procedure TFLaserLoc.BLogSaveClick(Sender: TObject);
begin
  if FileExistsUTF8(EditLogFile.Text) then DeleteFileUTF8(EditLogFile.Text);
  LogFile.SaveToFile(EditLogFile.Text);
end;

procedure TFLaserLoc.BLogClearClick(Sender: TObject);
begin
  LogFile.Clear;
  OdoRobotPose := RealRobotPose;
end;

procedure TFLaserLoc.UDPReceive(aSocket: TLSocket);
var data, s: string;
    NumberBytes: integer;
    id: char;
    i: integer;
    dx, dy: double;
    LaserInv: double;
    delta: int64;
    header: string;
    DistanceCalibration: boolean;
begin
  //Memo.Clear;
  UDP.GetMessage(data);

  ClearUDPBuffer(NetInBuf);
  NumberBytes := length(data);

  if (NumberBytes = 0) or (NumberBytes >= UDPBufSize) then
    exit;

  NetInBuf.MessSize := NumberBytes;
  NetInBuf.ReadDisp := 0;
  move(data[1], NetInBuf.data[0], NumberBytes);

  if RGLasers.ItemIndex = 2 then begin
    ProcessAccumulation();
    exit;
  //end else if RGLasers.ItemIndex = 3 then begin
  //  ProcessVlog();
  //  exit;
  end;

  ET.Clear;
  ET.Start;

  header := chr(NetGetByte(NetInBuf));
  header := header + chr(NetGetByte(NetInBuf));
  header := header + chr(NetGetByte(NetInBuf));

  if header = 'L62' then begin
    ENC_count := 0;
    inc(VLoc_count);
    inc(Laser_count);
    ENC1 := NetGetInt(NetInBuf);
    ENC2 := NetGetInt(NetInBuf);
    //MemoVLog.Lines.Add(format('ENC %d, %d', [enc1, enc2]));
    if CBVlogActive.Checked then begin
      s := 'OPL; ' + inttostr(GetTickCount) + '; ';
      s := s + format('%d; %g; %g; %g; ', [VLoc_count, VLocRobotPose.x, VLocRobotPose.y, VLocRobotPose.theta]);

      s := s + format('%d; %d; %d; ', [ENC_count, ENC1, ENC2]);

      s := s + format('%d; ', [Laser_count]);
      for i := laserFirstIdx to laserLastIdx do begin
        with LaserPoints[i] do begin
          s := s + FloatToStr(d) + '; ';
        end;
      end;
      VLogFile.Add(s);
    end;
    exit;
  end;

  if header = 'XYA' then begin
    VLoc_count := 0;
    VLocRobotPose.x := NetGetFloat(NetInBuf);
    VLocRobotPose.y := NetGetFloat(NetInBuf);
    VLocRobotPose.theta := NetGetFloat(NetInBuf);
    //MemoVLog.Lines.Add(format('XYA %g, %g, %g', [VLocRobotPose.x, VLocRobotPose.y, VLocRobotPose.theta]));
    exit;
  end;


  if RGLasers.ItemIndex = 0 then begin
    // we only expect packets from Hokuyo
    if header <> 'HA1' then exit;

    laserFirstIdx := 44;
    laserLastIdx := 725;
    SetLength(LaserPoints, 768);
    LaserAngleK1 := 270 / (3*256);
    LaserAngleK2 := -135;

  end else if RGLasers.ItemIndex = 1 then begin
    // we only expect packets from Neato
    if header <> 'NA1' then exit;

    laserFirstIdx := 0;
    laserLastIdx := 359;
    SetLength(LaserPoints, 360);
    LaserAngleK1 := 1;
    LaserAngleK2 := 0.5;
  end;

  if CBInvertedLaser.Checked then LaserInv := -1
  else LaserInv := 1;

  DistanceCalibration := CBLaserCalibration.Checked;

  Laser_count := 0;
  for i := laserFirstIdx to laserLastIdx do begin
    with LaserPoints[i] do begin
      d := netgetword(NetInBuf) / 1000;
      if DistanceCalibration then d := LaserDistanceC1 * d * d + LaserDistanceC2 * d + LaserDistanceC3;
      //angle := LaserInv * degtorad(270 * i / (3*256) - 135);// + LaserAngleOffset;
      angle := LaserInv * degtorad(LaserAngleK1 * i + LaserAngleK2) + LaserAngleOffset;
      x := d * cos(angle);
      y := d * sin(angle);
      std := 1;
    end;
  end;

  delta := round(1e6 * ET.Elapsed); // store the elapsed time
  EditTime.Text := format('%d', [delta]);
  ET.Clear;
  ET.Start;

  for i := 0 to MaxIters do begin
    FPaint.IterLaser(RobotPose, LaserPoints, laserFirstIdx, laserLastIdx, StepScale);
  end;
  EditFitError.Text := format('%.5g', [RobotPose.err]);

  delta := round(1e6 * ET.Elapsed); // store the elapsed time
  EditTime.Text := EditTime.Text + format(',pf: %d', [delta]);

  ET.Clear;
  ET.Start;

  //laserOffsetX := StrToFloatDef(EditLaserXOffset.text, 0);// 0.1;// - 0.015;
  dx := laserOffsetX * cos(RobotPose.theta);
  dy := laserOffsetX * sin(RobotPose.theta);

  if CBSendLock.Checked then SendLoc(EditSendLockIP.Text, SendLockPort, RobotPose);

  //EditRobotX.Text := format('%3f, %3f, %5f', [RobotPose.x - dx, RobotPose.y - dy, radtodeg(RobotPose.theta)]);

  EditRobotX.Text := format('%5f', [RobotPose.x - dx]);
  EditRobotY.Text := format('%5f', [RobotPose.y - dy]);
  EditRobotTheta.Text := format('%3f', [radtodeg(RobotPose.theta)]);

  if PageControl.ActivePage = TabLaser then begin
    CSLaser.Clear;
    for i := laserFirstIdx to laserLastIdx do begin
      CSLaser.AddXY(i, LaserPoints[i].d);
      //Memo.Lines.add(inttostr(d));
    end;
  end else if PageControl.ActivePage = TabLaserXY then begin
    CSLaserXY.Clear;
    for i := laserFirstIdx to laserLastIdx do begin
      CSLaserXY.AddXY(LaserPoints[i].x, LaserPoints[i].y);
    end;
  end;

  if CBShowLoc.Checked then FPaint.FormPaint(FPaint);

  delta := round(1e6 * ET.Elapsed); // store the elapsed time
  EditTime.Text :=  EditTime.Text + format(', %d us', [delta]);
  ET.Stop;
end;

procedure TFLaserLoc.ProcessAccumulation;
var id: char;
    i, n: integer;
    dx, dy, dtheta: double;
    LaserInv: double;
    delta: int64;
begin
  SetLength(LaserPoints, NumAccPoints);

  ET.Clear;
  ET.Start;

  // we only expect packets from Generic Measures
  if chr(NetGetByte(NetInBuf)) <> 'G' then
    exit;
  if chr(NetGetByte(NetInBuf)) <> 'A' then
    exit;
  if chr(NetGetByte(NetInBuf)) <> '1' then
    exit;

  laserLastIdx := NetGetWord(NetInBuf) - 1;// Read number of points
  laserFirstIdx := 0;

  SetLength(TmpLaserPoints, laserLastIdx + 1);

  if CBInvertedLaser.Checked then LaserInv := -1
  else LaserInv := 1;

  for i := 0 to laserLastIdx do begin
    with TmpLaserPoints[i] do begin
      x := NetGetInt(NetInBuf) / 1000;
      y := NetGetInt(NetInBuf) / 1000;
      d := sqrt(sqr(x) + sqr(y));
      angle := atan2(y, x);
      //CSLaserXY.AddXY(x, y);
    end;
  end;

  if CBAccumulation.Checked then begin
    tcount := NetGetWord(NetInBuf); // Get time
    odo1 := NetGetFloat(NetInBuf); // Get odo1
    odo2 := NetGetFloat(NetInBuf); // Get odo2
    RealRobotV := 4.36e-4 * (odo1 + odo2)/dt;
    RealRobotW := 3.9e-4 * (odo2 - odo1)/(0.1255 * dt);
    //RealRobotV := NetGetFloat(NetInBuf); // Get speed
    //RealRobotW := NetGetFloat(NetInBuf); // Get angular speed

    RealRobotPose.x := NetGetFloat(NetInBuf); // Get Robot x
    RealRobotPose.y := NetGetFloat(NetInBuf); // Get Robot y
    RealRobotPose.theta := NetGetFloat(NetInBuf); // Get Robot theta

    //RobotPose := RealRobotPose;
    RobotPose.x := RobotPose.x + dt * RealRobotV * cos(RobotPose.theta);
    RobotPose.y := RobotPose.y + dt * RealRobotV * sin(RobotPose.theta);
    RobotPose.theta := RobotPose.theta + dt * RealRobotW;

    with OdoRobotPose do begin
      x := x + dt * RealRobotV * cos(theta);
      y := y + dt * RealRobotV * sin(theta);
      theta := theta + dt * RealRobotW;
      //Memo.Lines.add(format('%.4g %.4g %.4g',[x, y, theta]));
      //if Memo.Lines.Count > 10 then memo.lines.Delete(0);
    end;

    // Bring old points to current position using odometry
    for i := 0 to Length(LaserPoints) - 1 do begin
      with LaserPoints[i] do begin
        dtheta := - dt * RealRobotW;
        dx := - dt * RealRobotV;
        dy := 0;
        //TranslateAndRotate(x, y, x, y, dx, dy, dtheta);
        RotateAndTranslate(x, y, x, y, dx, dy, dtheta);
        angle := angle + dtheta;
      end;
    end;

    // insert the new points
    // make room
    for i := Length(TmpLaserPoints) to Length(LaserPoints) - 1 do begin
      LaserPoints[i - Length(TmpLaserPoints)] := LaserPoints[i];
    end;
    // insert
    for i := 0 to Length(TmpLaserPoints) - 1 do begin
      //LaserPoints[(Length(LaserPoints) - 1) - Length(TmpLaserPoints) + i] := TmpLaserPoints[i];
      LaserPoints[(Length(LaserPoints) - 1) - i] := TmpLaserPoints[i];
    end;

  end else begin // Normal processing without accumulation
    LaserPoints := TmpLaserPoints;
  end;

  delta := round(1e6 * ET.Elapsed); // store the elapsed time
  EditTime.Text := format('%d', [delta]);

  for i := 0 to MaxIters do begin
    FPaint.IterLaser(RobotPose, LaserPoints, 0, Length(LaserPoints) - 1, StepScale);
  end;
  EditFitError.Text := format('%.5g', [RobotPose.err]);

  delta := round(1e6 * ET.Elapsed); // store the elapsed time
  EditTime.Text := EditTime.Text + format(', %d', [delta]);

  //laserOffsetX := StrToFloatDef(EditLaserXOffset.text, 0);// 0.1;// - 0.015;
  dx := laserOffsetX * cos(RobotPose.theta);
  dy := laserOffsetX * sin(RobotPose.theta);

  if CBSendLock.Checked then SendLoc(EditSendLockIP.Text, SendLockPort, RobotPose);

  EditRobotX.Text := format('%.3g', [RobotPose.x - dx]);
  EditRobotY.Text := format('%.3g', [RobotPose.y - dy]);
  EditRobotTheta.Text := format('%.5g', [radtodeg(RobotPose.theta)]);

  if PageControl.ActivePage = TabLaser then begin
    CSLaser.Clear;
    for i := laserFirstIdx to laserLastIdx do begin
      CSLaser.AddXY(i, LaserPoints[i].d);
      //Memo.Lines.add(inttostr(d));
    end;
  end else if PageControl.ActivePage = TabLaserXY then begin
    CSLaserXY.Clear;
    for i := laserFirstIdx to laserLastIdx do begin
      CSLaserXY.AddXY(LaserPoints[i].x, LaserPoints[i].y);
    end;
  end;

  if CBShowLoc.Checked then FPaint.FormPaint(FPaint);

  delta := round(1e6 * ET.Elapsed); // store the elapsed time
  EditTime.Text :=  EditTime.Text + format(', %d us', [delta]);
  ET.Stop;

  if CBAccumulation.Checked then begin
    LogFile.Add(Format('%d %g %g  %g %g %g  %g %g %g  %g %g %g',
                       [tcount, RealRobotV, RealRobotW,
                        RealRobotPose.x, RealRobotPose.y, RealRobotPose.theta,
                        RobotPose.x, RobotPose.y, RobotPose.theta,
                        OdoRobotPose.x, OdoRobotPose.y, OdoRobotPose.theta
                       ]));
  end;

end;


procedure TFLaserLoc.ProcessVLog;
var id: char;
    i, n, idx: integer;
    dx, dy, dtheta: double;
    delta: int64;
    s: string;
    sl :TStringList;
    ticks: DWord;
    LaserInv: integer;
    b1, b2: double;
begin
  //SetLength(LaserPoints, NumAccPoints);

  ET.Clear;
  ET.Start;

  s := VLogFile.Strings[VLogLineNum];
  sl := TStringList.Create;
  try
    ParseString(s, '; ', sl);
    idx := 0;
    //Memo.Lines.AddStrings(sl);
    //VLogLineNum := VLogFile.Count;
    n := VLogLineNum;
    ticks := StrToInt64(sl[1]);
    VLoc_count := StrToInt(sl[2]);
    VLocRobotPose.x := StrToFloat(sl[3]);
    VLocRobotPose.y := StrToFloat(sl[4]);
    VLocRobotPose.theta := StrToFloat(sl[5]);

    ENC_count := StrToInt(sl[6]);
    ENC1 := StrToInt(sl[7]);
    ENC2 := StrToInt(sl[8]);

    Laser_count := StrToInt(sl[9]);

    if CBInvertedLaser.Checked then LaserInv := -1
    else LaserInv := 1;

    laserFirstIdx := 0;
    laserLastIdx := sl.Count - (10 + 1);

    //laserLastIdx := 359;
    SetLength(LaserPoints, laserLastIdx - laserFirstIdx + 1);
    LaserAngleK1 := 1; // TODO: more generic, now it is set for the neato
    LaserAngleK2 := 0.5;

    b1 := 0.001523985;
    b2 := 1.3102842636;
    Laser_count := 0;
    for i := laserFirstIdx to laserLastIdx do begin
      with LaserPoints[i] do begin
        d := StrToFloat(sl[10 + i]);
        angle := LaserInv * degtorad(LaserAngleK1 * i + LaserAngleK2) + LaserAngleOffset;
        x := d * cos(angle);
        y := d * sin(angle);
        if CBStdevComp.Checked then begin
          std := (b1 * exp( b2 * D)) / (b1 * exp( b2 * 1));
        end else begin
          std := 1;
        end;
      end;
    end;

    delta := round(1e6 * ET.Elapsed); // store the elapsed time
    EditTime.Text := format('%d', [delta]);

    for i := 0 to MaxIters do begin
      FPaint.IterLaser(RobotPose, LaserPoints, 0, Length(LaserPoints) - 1, StepScale);
    end;
    EditFitError.Text := format('%.5g', [RobotPose.err]);

    delta := round(1e6 * ET.Elapsed); // store the elapsed time
    EditTime.Text := EditTime.Text + format(', %d', [delta]);

    //laserOffsetX := StrToFloatDef(EditLaserXOffset.text, 0);// 0.1;// - 0.015;
    dx := laserOffsetX * cos(RobotPose.theta);
    dy := laserOffsetX * sin(RobotPose.theta);

    if CBSendLock.Checked then SendLoc(EditSendLockIP.Text, SendLockPort, RobotPose);

    with RobotPoseMSE do begin
      x := (n * x + abs(VLocRobotPose.x - (RobotPose.x - dx)))/(n + 1);
      y := (n * y + abs(VLocRobotPose.y - (RobotPose.y - dy)))/(n + 1);
      theta := (n * theta + abs(DiffAngle(RobotPose.theta, VLocRobotPose.theta)))/(n + 1);
    end;

    EditRobotX.Text := format('%.3g', [RobotPose.x - dx]);
    EditRobotY.Text := format('%.3g', [RobotPose.y - dy]);
    EditRobotTheta.Text := format('%.5g', [radtodeg(RobotPose.theta)]);

    EditRobotXVLoc.Text := format('%.3g', [VLocRobotPose.x]);
    EditRobotYVLoc.Text := format('%.3g', [VLocRobotPose.y]);
    EditRobotThetaVLoc.Text := format('%.5g', [radtodeg(VLocRobotPose.theta)]);

    EditRobotXErr.Text := format('%.3g', [RobotPoseMSE.x]);
    EditRobotYErr.Text := format('%.3g', [RobotPoseMSE.y]);
    EditRobotThetaErr.Text := format('%.5g', [radtodeg(RobotPoseMSE.theta)]);

    if PageControl.ActivePage = TabLaser then begin
      CSLaser.Clear;
      for i := laserFirstIdx to laserLastIdx do begin
        CSLaser.AddXY(i, LaserPoints[i].d);
        //Memo.Lines.add(inttostr(d));
      end;
    end else if PageControl.ActivePage = TabLaserXY then begin
      CSLaserXY.Clear;
      for i := laserFirstIdx to laserLastIdx do begin
        CSLaserXY.AddXY(LaserPoints[i].x, LaserPoints[i].y);
      end;
    end;

    if CBShowLoc.Checked then FPaint.FormPaint(FPaint);

    delta := round(1e6 * ET.Elapsed); // store the elapsed time
    EditTime.Text :=  EditTime.Text + format(', %d us', [delta]);
    ET.Stop;

    if CBAccumulation.Checked then begin
      LogFile.Add(Format('%d %g %g  %g %g %g  %g %g %g  %g %g %g',
                         [tcount, RealRobotV, RealRobotW,
                          RealRobotPose.x, RealRobotPose.y, RealRobotPose.theta,
                          RobotPose.x, RobotPose.y, RobotPose.theta,
                          OdoRobotPose.x, OdoRobotPose.y, OdoRobotPose.theta
                         ]));
    end;

  finally
    sl.Free;
  end;
end;



procedure TFLaserLoc.SendLoc(ToIP: string; port: integer; Loc: TPose);
var i, start: integer;
begin
    //if not UDP.Connected then exit;

    ClearUDPBuffer(NetOutBuf);
    // tag this packet as a Localization Event
    NetPutByte(NetOutBuf, ord('L'));
    NetPutByte(NetOutBuf, ord('O'));
    NetPutByte(NetOutBuf, ord('C'));
    NetPutByte(NetOutBuf, ord('0'));

    NetPutFloat(NetOutBuf, Loc.x);
    NetPutFloat(NetOutBuf, Loc.y);
    NetPutFloat(NetOutBuf, Loc.theta);

    UDPSend.Send(NetOutBuf.data, NetOutBuf.MessSize, ToIP);
end;

initialization
  {$I laserloc.lrs}

end.

