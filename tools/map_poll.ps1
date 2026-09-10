<#
.SYNOPSIS
    Odpytuje wezly o mape sieci i dopisuje wszystko, co powiedza, do jednego logu.

.DESCRIPTION
    Zeby narysowac CALA trase, a nie tylko pierwszy skok, potrzebne sa tablice
    routingu z kazdego posrednika. Ten skrypt otwiera kilka portow naraz, co
    kilkanascie sekund wysyla na kazdy komende MAP i zapisuje odpowiedzi do
    wspolnego pliku. map_graph.py --follow czyta ten plik i przerysowuje mape.

    Uzywa System.IO.Ports z .NET, wiec nie wymaga pyserial ani niczego do
    doinstalowania.

    UWAGA NA PORT ZAJETY. Windows daje port jednemu procesowi. Jesli programator
    w Javie trzyma port wezla, ten skrypt go nie otworzy - i odwrotnie. Wtedy
    loguj konsole Javy do pliku i podaj ten plik skryptowi rysujacemu, albo
    najpierw zamknij Jave.

    DTR zostaje OPUSZCZONY celowo: tam, gdzie ta linia jest wpieta w reset,
    podniesienie jej restartuje wezel, a logowanie nie ma tego robic.

    JEDEN WEZEL TO JEDEN SKOK. Ze zrzutu z jednego portu odczytasz mape jego
    okolicy i PIERWSZY skok kazdej trasy - dalszej drogi ten wezel nie zna.
    Cala trasa wymaga podania tu portow wszystkich posrednikow.

.EXAMPLE
    .\map_poll.ps1 -Ports COM5 -LogFile Z:\logi\mapa.log

.EXAMPLE
    .\map_poll.ps1 -Ports COM5 -NoQuery
    Tylko nasluch, bez wysylania MAP - gdy o mape pyta juz ktos inny.
#>
param(
    [string[]]$Ports = @("COM5"),
    [int]$BaudRate = 115200,
    [string]$LogFile = "mapa.log",
    [int]$IntervalSeconds = 15,
    [switch]$NoQuery
)

$ErrorActionPreference = "Continue"

$open = @{}
$partial = @{}
foreach ($name in $Ports) {
    $port = New-Object System.IO.Ports.SerialPort $name, $BaudRate, "None", 8, "One"
    $port.DtrEnable = $false   # DTR = reset Moteino, wiec ani go nie dotykamy
    $port.RtsEnable = $false
    $port.ReadTimeout = 200
    $port.NewLine = "`n"
    try {
        $port.Open()
        $open[$name] = $port
        $partial[$name] = ""
        Write-Host "otwarty $name"
    } catch {
        Write-Host "nie moge otworzyc $name : $($_.Exception.Message)"
        Write-Host "  (port zajety? zamknij programator w Javie albo terminal)"
    }
}

if ($open.Count -eq 0) {
    Write-Host "zaden port sie nie otworzyl - koncze"
    exit 1
}

$header = "=== START {0} ===" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss")
Add-Content -Path $LogFile -Value $header -Encoding utf8
Write-Host "loguje do $LogFile, przerwij Ctrl+C"
if (-not $NoQuery) {
    Write-Host "pytam o mape co $IntervalSeconds s"
}

# Pierwsze zapytanie od razu, kolejne co IntervalSeconds.
$nextQuery = Get-Date

try {
    while ($true) {
        if (-not $NoQuery -and (Get-Date) -ge $nextQuery) {
            foreach ($name in @($open.Keys)) {
                try {
                    $open[$name].WriteLine("MAP")
                } catch {
                    Write-Host "$name : nie udalo sie wyslac MAP ($($_.Exception.Message))"
                }
            }
            $nextQuery = (Get-Date).AddSeconds($IntervalSeconds)
        }

        foreach ($name in @($open.Keys)) {
            $chunk = ""
            try {
                $chunk = $open[$name].ReadExisting()
            } catch {
                $chunk = ""
            }
            if ([string]::IsNullOrEmpty($chunk)) { continue }

            # Sklejamy z ogonem z poprzedniego odczytu - port oddaje dane w kawalkach,
            # ktore potrafia sie urwac w polowie linii.
            $buffer = $partial[$name] + $chunk
            $lines = $buffer -split "`r?`n"
            $partial[$name] = $lines[-1]
            if ($lines.Count -gt 1) {
                $stamp = Get-Date -Format "HH:mm:ss"
                $complete = $lines[0..($lines.Count - 2)]
                $out = foreach ($line in $complete) {
                    if ($line.Trim().Length -gt 0) { "$stamp [$name] $line" }
                }
                if ($out) {
                    Add-Content -Path $LogFile -Value $out -Encoding utf8
                    foreach ($line in $out) {
                        if ($line -match "MAP ") { Write-Host $line }
                    }
                }
            }
        }
        Start-Sleep -Milliseconds 100
    }
} finally {
    foreach ($name in @($open.Keys)) {
        try { $open[$name].Close() } catch { }
    }
    Write-Host "porty zamkniete"
}
