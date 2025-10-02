# Makefile для проекта SmartKayak PlatformIO
# Упрощает работу с PlatformIO командами

# Определение окружений
PADDLE_ENV = paddle_esp32c3_mini
KAYAK_ENV = kayak_smart_esp32

# Цвета для вывода
COLOR_RESET = \033[0m
COLOR_INFO = \033[1;34m
COLOR_SUCCESS = \033[1;32m
COLOR_WARNING = \033[1;33m

# По умолчанию показываем помощь
.DEFAULT_GOAL := help

.PHONY: help
help: ## Показать это сообщение помощи
	@echo "$(COLOR_INFO)Доступные команды:$(COLOR_RESET)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  $(COLOR_SUCCESS)%-20s$(COLOR_RESET) %s\n", $$1, $$2}'

# ============================================
# Команды для весла (Paddle)
# ============================================

.PHONY: paddle-build
paddle-build: ## Собрать прошивку для весла
	@echo "$(COLOR_INFO)Сборка прошивки для весла...$(COLOR_RESET)"
	pio run -e $(PADDLE_ENV)

.PHONY: paddle-upload
paddle-upload: ## Загрузить прошивку в весло
	@echo "$(COLOR_INFO)Загрузка прошивки в весло...$(COLOR_RESET)"
	pio run -e $(PADDLE_ENV) -t upload

.PHONY: paddle-monitor
paddle-monitor: ## Открыть монитор для весла
	@echo "$(COLOR_INFO)Открытие монитора для весла...$(COLOR_RESET)"
	pio device monitor -e $(PADDLE_ENV)

.PHONY: paddle-all
paddle-all: paddle-build paddle-upload paddle-monitor ## Собрать, загрузить и открыть монитор для весла

.PHONY: paddle-clean
paddle-clean: ## Очистить сборку весла
	@echo "$(COLOR_INFO)Очистка сборки весла...$(COLOR_RESET)"
	pio run -e $(PADDLE_ENV) -t clean

# ============================================
# Команды для каяка (Kayak)
# ============================================

.PHONY: kayak-build
kayak-build: ## Собрать прошивку для каяка
	@echo "$(COLOR_INFO)Сборка прошивки для каяка...$(COLOR_RESET)"
	pio run -e $(KAYAK_ENV)

.PHONY: kayak-upload
kayak-upload: ## Загрузить прошивку в каяк
	@echo "$(COLOR_INFO)Загрузка прошивки в каяк...$(COLOR_RESET)"
	pio run -e $(KAYAK_ENV) -t upload

.PHONY: kayak-monitor
kayak-monitor: ## Открыть монитор для каяка
	@echo "$(COLOR_INFO)Открытие монитора для каяка...$(COLOR_RESET)"
	pio device monitor -e $(KAYAK_ENV)

.PHONY: kayak-all
kayak-all: kayak-build kayak-upload kayak-monitor ## Собрать, загрузить и открыть монитор для каяка

.PHONY: kayak-clean
kayak-clean: ## Очистить сборку каяка
	@echo "$(COLOR_INFO)Очистка сборки каяка...$(COLOR_RESET)"
	pio run -e $(KAYAK_ENV) -t clean

# ============================================
# Общие команды
# ============================================

.PHONY: build-all
build-all: ## Собрать прошивки для всех устройств
	@echo "$(COLOR_INFO)Сборка всех прошивок...$(COLOR_RESET)"
	pio run

.PHONY: clean-all
clean-all: ## Очистить все сборки
	@echo "$(COLOR_INFO)Очистка всех сборок...$(COLOR_RESET)"
	pio run -t clean

.PHONY: update-libs
update-libs: ## Обновить все библиотеки
	@echo "$(COLOR_INFO)Обновление библиотек...$(COLOR_RESET)"
	pio pkg update

.PHONY: install-libs
install-libs: ## Установить все зависимости
	@echo "$(COLOR_INFO)Установка зависимостей...$(COLOR_RESET)"
	pio pkg install

.PHONY: list-libs
list-libs: ## Показать список установленных библиотек
	@echo "$(COLOR_INFO)Список установленных библиотек:$(COLOR_RESET)"
	pio pkg list

.PHONY: check
check: ## Проверить код (статический анализ)
	@echo "$(COLOR_INFO)Проверка кода...$(COLOR_RESET)"
	pio check

.PHONY: test
test: ## Запустить тесты
	@echo "$(COLOR_INFO)Запуск тестов...$(COLOR_RESET)"
	pio test

# ============================================
# Утилиты
# ============================================

.PHONY: devices
devices: ## Показать подключенные устройства
	@echo "$(COLOR_INFO)Подключенные устройства:$(COLOR_RESET)"
	pio device list

.PHONY: info
info: ## Показать информацию о проекте
	@echo "$(COLOR_INFO)Информация о проекте:$(COLOR_RESET)"
	@echo "  Окружения:"
	@echo "    - $(PADDLE_ENV) (весло ESP32-C3)"
	@echo "    - $(KAYAK_ENV) (каяк ESP32)"
	@echo ""
	@pio project config

.PHONY: fullclean
fullclean: ## Полная очистка (включая .pio каталог)
	@echo "$(COLOR_WARNING)Полная очистка проекта...$(COLOR_RESET)"
	rm -rf .pio

.PHONY: format
format: ## Форматировать код (если установлен clang-format)
	@echo "$(COLOR_INFO)Форматирование кода...$(COLOR_RESET)"
	@find src lib -name "*.cpp" -o -name "*.h" | xargs clang-format -i 2>/dev/null || echo "clang-format не установлен"

# ============================================
# Дополнительные цели
# ============================================

.PHONY: paddle-debug
paddle-debug: ## Сборка с отладочной информацией для весла
	@echo "$(COLOR_INFO)Сборка с отладкой для весла...$(COLOR_RESET)"
	pio run -e $(PADDLE_ENV) -v

.PHONY: kayak-debug
kayak-debug: ## Сборка с отладочной информацией для каяка
	@echo "$(COLOR_INFO)Сборка с отладкой для каяка...$(COLOR_RESET)"
	pio run -e $(KAYAK_ENV) -v

.PHONY: size
size: ## Показать размер прошивок
	@echo "$(COLOR_INFO)Размер прошивок:$(COLOR_RESET)"
	@echo "$(COLOR_SUCCESS)Весло:$(COLOR_RESET)"
	@pio run -e $(PADDLE_ENV) -t size 2>/dev/null || echo "  Сначала выполните сборку"
	@echo "$(COLOR_SUCCESS)Каяк:$(COLOR_RESET)"
	@pio run -e $(KAYAK_ENV) -t size 2>/dev/null || echo "  Сначала выполните сборку"

