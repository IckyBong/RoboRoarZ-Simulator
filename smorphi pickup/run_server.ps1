# Smorphi Robotics Simulator - Local Launcher Script
# Starts a lightweight HTTP server on port 8080 and opens the browser.

$port = 8080
$url = "http://localhost:$port"
$directory = $PSScriptRoot

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host "   SMORPHI ROBOROARZ AUTONOMOUS ROBOTICS SIMULATOR        " -ForegroundColor Yellow
Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host "Serving directory: $directory" -ForegroundColor Gray
Write-Host "Opening simulator at: $url" -ForegroundColor Green

# Launch browser
Start-Process $url

# Simple PowerShell HTTP Server
$listener = New-Object System.Net.HttpListener
$listener.Prefixes.Add("http://localhost:$port/")

try {
    $listener.Start()
    Write-Host "Server running at http://localhost:$port/ (Press Ctrl+C to stop)..." -ForegroundColor White

    while ($listener.IsListening) {
        $context = $listener.GetContext()
        $request = $context.Request
        $response = $context.Response

        $localPath = $request.Url.LocalPath.TrimStart('/')
        if ([string]::IsNullOrEmpty($localPath)) {
            $localPath = "index.html"
        }

        $filePath = Join-Path $directory $localPath

        if (Test-Path $filePath -PathType Leaf) {
            $content = [System.IO.File]::ReadAllBytes($filePath)
            
            # Content-Type Mapping
            $ext = [System.IO.Path]::GetExtension($filePath).ToLower()
            $mime = "text/plain"
            switch ($ext) {
                ".html" { $mime = "text/html; charset=utf-8" }
                ".css"  { $mime = "text/css; charset=utf-8" }
                ".js"   { $mime = "application/javascript; charset=utf-8" }
                ".json" { $mime = "application/json; charset=utf-8" }
                ".png"  { $mime = "image/png" }
                ".jpg"  { $mime = "image/jpeg" }
                ".svg"  { $mime = "image/svg+xml" }
            }

            $response.ContentType = $mime
            $response.ContentLength64 = $content.Length
            $response.OutputStream.Write($content, 0, $content.Length)
        } else {
            $response.StatusCode = 404
            $notFound = [System.Text.Encoding]::UTF8.GetBytes("404 Not Found")
            $response.ContentLength64 = $notFound.Length
            $response.OutputStream.Write($notFound, 0, $notFound.Length)
        }

        $response.Close()
    }
} catch {
    Write-Host "Server stopped." -ForegroundColor Gray
} finally {
    $listener.Stop()
}
