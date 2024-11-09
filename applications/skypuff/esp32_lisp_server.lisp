; STR500 IO GPIO Example

; Add these 3 lines to your ESC's Lisp Script if they are not already present
;(import "pkg@://vesc_packages/lib_code_server/code_server.vescpkg" 'code-server)
;(read-eval-program code-server)
;(start-code-server)


(gpio-configure 8 'pin-mode-out)
(gpio-write 8 0)

(import "pkg@://vesc_packages/lib_code_server/code_server.vescpkg" 'code-server)
(read-eval-program code-server)

(start-code-server)
(print "Code server started")
